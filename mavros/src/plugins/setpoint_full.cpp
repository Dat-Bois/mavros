/**
 * @brief FullSetpoint plugin
 * @file setpoint_full.cpp
 * @author Eesh Vij <evij@uci.edu>
 *
 * @addtogroup plugin
 * @{
 */

#include "tf2_eigen/tf2_eigen.hpp"
#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"
#include "mavros/setpoint_mixin.hpp"

#include "mavros_msgs/msg/full_setpoint.hpp"
#include "geometry_msgs/msg/vector3.hpp"

namespace mavros
{
namespace std_plugins
{
using namespace std::placeholders;      // NOLINT

/**
 * @brief Full setpoint plugin
 * @plugin setpoint_full
 *
 * Send a full localNED setpoint to FCU controller.
 */
class SetpointFullPlugin : public plugin::Plugin,
  private plugin::SetPositionTargetLocalNEDMixin<SetpointFullPlugin>
{
public:
  explicit SetpointFullPlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "setpoint_full")
  {
    auto sensor_qos = rclcpp::SensorDataQoS();

    fullset_sub = node->create_subscription<mavros_msgs::msg::FullSetpoint>(
      "~/cmd_full_setpoint", sensor_qos, std::bind(
        &SetpointFullPlugin::setpoint_cb, this,
        _1));
  }

  Subscriptions get_subscriptions() override
  {
    return { /* Rx disabled */};
  }

private:
  friend class plugin::SetPositionTargetLocalNEDMixin<SetpointFullPlugin>;

  rclcpp::Subscription<mavros_msgs::msg::FullSetpoint>::SharedPtr fullset_sub;

  /* -*- mid-level helpers -*- */

  /**
   * @brief Send a full localNED setpoint to FCU controller.
   *
   */
  void send_setpoint_full(const rclcpp::Time & stamp, const uint16_t type_mask,
                          const Eigen::Vector3d & pos_enu,
                          const Eigen::Vector3d & vel_enu,
                          const Eigen::Vector3d & accel_enu,
                          const double yaw, const double yaw_rate)
  {
    using mavlink::common::MAV_FRAME;


    auto pos_ned = [&]() -> Eigen::Vector3d {
      // Check if position is ignored in the type_mask (rightmost 3 bits)
      if ((type_mask & 0x7) == 0x7) {
        return Eigen::Vector3d::Zero(); // Ignore position
      }
      return ftf::transform_frame_enu_ned(pos_enu);
    } ();

    auto vel_ned = [&]() -> Eigen::Vector3d {
      // Check if velocity is ignored in the type_mask (bits 3-5)
      if ((type_mask & (0x7 << 3)) == (0x7 << 3)) {
        return Eigen::Vector3d::Zero(); // Ignore velocity
      }
      return ftf::transform_frame_enu_ned(vel_enu);
    } ();

    auto accel_ned = [&]() -> Eigen::Vector3d {
      // Check if acceleration is ignored in the type_mask (bits 6-8)
      if ((type_mask & (0x7 << 6)) == (0x7 << 6)) {
        return Eigen::Vector3d::Zero(); // Ignore acceleration
      }
      return ftf::transform_frame_enu_ned(accel_enu);
    } ();

    // Check if yaw is ignored in the type_mask (bit 10)
    auto y = [&]() {
      if ((type_mask & (1 << 10)) != 0) {
        return 0.0; // Ignore yaw
      }
      // return ftf::quaternion_get_yaw(ftf::transform_orientation_enu_ned(
      //   ftf::transform_orientation_baselink_aircraft(
      //     Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())))));
      return yaw;
    } ();

    auto yr = [&]() {
      // Check if yaw rate is ignored in the type_mask (bit 11)
      if ((type_mask & (1 << 11)) != 0) {
        return 0.0; // Ignore yaw rate
      }
      return ftf::transform_frame_ned_enu(Eigen::Vector3d(0.0, 0.0, yaw_rate)).z();
    } ();

    set_position_target_local_ned(
      get_time_boot_ms(stamp),
      utils::enum_value(MAV_FRAME::LOCAL_NED),
      type_mask,
      pos_ned,
      vel_ned,
      accel_ned,
      y, yr);
  }

  /* -*- callbacks -*- */

  void setpoint_cb(const mavros_msgs::msg::FullSetpoint::SharedPtr req)
  {
    Eigen::Vector3d pos_enu;
    Eigen::Vector3d vel_enu;
    Eigen::Vector3d accel_enu;

    tf2::fromMsg(req->position, pos_enu);
    tf2::fromMsg(req->velocity, vel_enu);
    tf2::fromMsg(req->acceleration, accel_enu);
    send_setpoint_full(
      req->header.stamp,
      req->type_mask,
      pos_enu,
      vel_enu,
      accel_enu,
      req->yaw,
      req->yaw_rate);
  }
};

}       // namespace std_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::std_plugins::SetpointFullPlugin)

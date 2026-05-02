// Copyright 2026 Yutaka Kondo
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "kachaka_autoware_vehicle_interface/vehicle_interface_node.hpp"

#include <chrono>
#include <memory>
#include <utility>

namespace kachaka_autoware_vehicle_interface
{

using namespace std::chrono_literals;
using OperationModeStateMsg = autoware_adapi_v1_msgs::msg::OperationModeState;

VehicleInterfaceNode::VehicleInterfaceNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("kachaka_autoware_vehicle_interface", options),
  last_control_stamp_(0, 0, RCL_ROS_TIME)
{
  ControlToTwistParams params;
  params.wheel_base = declare_parameter<double>("wheel_base", 0.30);
  params.max_linear_velocity = declare_parameter<double>("max_linear_velocity", 0.3);
  params.max_angular_velocity = declare_parameter<double>("max_angular_velocity", 1.57);
  cmd_vel_timeout_sec_ = declare_parameter<double>("cmd_vel_timeout", 0.5);
  const double velocity_status_period =
    declare_parameter<double>("publish_period_velocity_status", 0.02);
  const double op_mode_period =
    declare_parameter<double>("publish_period_operation_mode", 0.1);
  const bool auto_enable = declare_parameter<bool>("auto_enable_manual_control", true);
  converter_ = std::make_unique<ControlToTwistConverter>(params);

  control_sub_ = create_subscription<autoware_control_msgs::msg::Control>(
    "/control/command/control_cmd", rclcpp::QoS(1),
    std::bind(&VehicleInterfaceNode::on_control, this, std::placeholders::_1));
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "/kachaka/wheel_odometry/wheel_odometry", rclcpp::SensorDataQoS(),
    std::bind(&VehicleInterfaceNode::on_odom, this, std::placeholders::_1));

  twist_pub_ = create_publisher<geometry_msgs::msg::Twist>(
    "/kachaka/manual_control/cmd_vel", rclcpp::SensorDataQoS());
  velocity_status_pub_ = create_publisher<autoware_vehicle_msgs::msg::VelocityReport>(
    "/vehicle/status/velocity_status", rclcpp::QoS(1));
  op_mode_pub_ = create_publisher<OperationModeStateMsg>(
    "/system/operation_mode/state", rclcpp::QoS(1).transient_local());

  change_to_autonomous_srv_ =
    create_service<autoware_adapi_v1_msgs::srv::ChangeOperationMode>(
    "/system/operation_mode/change_to_autonomous",
    std::bind(
      &VehicleInterfaceNode::on_change_to_autonomous, this,
      std::placeholders::_1, std::placeholders::_2));
  change_to_stop_srv_ =
    create_service<autoware_adapi_v1_msgs::srv::ChangeOperationMode>(
    "/system/operation_mode/change_to_stop",
    std::bind(
      &VehicleInterfaceNode::on_change_to_stop, this,
      std::placeholders::_1, std::placeholders::_2));

  enable_manual_control_client_ =
    create_client<std_srvs::srv::SetBool>("/kachaka/manual_control/set_enabled");

  velocity_status_timer_ = create_wall_timer(
    std::chrono::duration<double>(velocity_status_period),
    std::bind(&VehicleInterfaceNode::on_velocity_status_timer, this));
  op_mode_timer_ = create_wall_timer(
    std::chrono::duration<double>(op_mode_period),
    std::bind(&VehicleInterfaceNode::on_op_mode_timer, this));
  cmd_vel_timeout_timer_ = create_wall_timer(
    100ms, std::bind(&VehicleInterfaceNode::on_cmd_vel_timeout_timer, this));

  if (auto_enable) {
    enable_manual_control(true);
  }

  RCLCPP_INFO(
    get_logger(),
    "VehicleInterfaceNode started: wheel_base=%.3f, vmax=%.3f, wmax=%.3f, timeout=%.2fs",
    params.wheel_base, params.max_linear_velocity, params.max_angular_velocity,
    cmd_vel_timeout_sec_);
}

void VehicleInterfaceNode::on_control(
  const autoware_control_msgs::msg::Control::SharedPtr msg)
{
  // Only stamp on AUTONOMOUS-period traffic — the watchdog is "control_cmd
  // stops arriving while AUTONOMOUS". Stamping unconditionally would let
  // STOP-mode chatter reset the watchdog and silently delay the zero-Twist
  // failsafe by up to cmd_vel_timeout after the AUTONOMOUS transition.
  if (state_machine_.get_state() != OperationMode::AUTONOMOUS) {
    return;
  }
  last_control_stamp_ = now();
  twist_pub_->publish(converter_->convert(*msg));
}

void VehicleInterfaceNode::on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  latest_odom_ = msg;
}

void VehicleInterfaceNode::on_velocity_status_timer()
{
  if (!latest_odom_) {
    return;
  }
  velocity_status_pub_->publish(convert_odometry_to_velocity_report(*latest_odom_));
}

void VehicleInterfaceNode::on_op_mode_timer()
{
  OperationModeStateMsg state;
  state.stamp = now();
  state.mode = (state_machine_.get_state() == OperationMode::AUTONOMOUS) ?
    OperationModeStateMsg::AUTONOMOUS :
    OperationModeStateMsg::STOP;
  state.is_autoware_control_enabled = (state.mode == OperationModeStateMsg::AUTONOMOUS);
  state.is_in_transition = false;
  state.is_stop_mode_available = true;
  state.is_autonomous_mode_available = true;
  state.is_local_mode_available = false;
  state.is_remote_mode_available = false;
  op_mode_pub_->publish(state);
}

void VehicleInterfaceNode::on_cmd_vel_timeout_timer()
{
  if (state_machine_.get_state() != OperationMode::AUTONOMOUS) {
    return;
  }
  // Skip until the first control message has arrived (last_control_stamp_ == 0).
  if (last_control_stamp_.nanoseconds() == 0) {
    return;
  }
  const auto elapsed = (now() - last_control_stamp_).seconds();
  if (elapsed > cmd_vel_timeout_sec_) {
    geometry_msgs::msg::Twist zero;
    twist_pub_->publish(zero);
  }
}

void VehicleInterfaceNode::on_change_to_autonomous(
  const autoware_adapi_v1_msgs::srv::ChangeOperationMode::Request::SharedPtr /*req*/,
  autoware_adapi_v1_msgs::srv::ChangeOperationMode::Response::SharedPtr resp)
{
  // Reset the watchdog stamp so a stale timestamp from a prior AUTONOMOUS
  // session cannot trip the zero-Twist failsafe before the first new
  // control_cmd is received.
  last_control_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  state_machine_.request_autonomous();
  resp->status.success = true;
}

void VehicleInterfaceNode::on_change_to_stop(
  const autoware_adapi_v1_msgs::srv::ChangeOperationMode::Request::SharedPtr /*req*/,
  autoware_adapi_v1_msgs::srv::ChangeOperationMode::Response::SharedPtr resp)
{
  state_machine_.request_stop();
  resp->status.success = true;
}

void VehicleInterfaceNode::enable_manual_control(bool enable)
{
  // Non-blocking: arm a 500 ms periodic timer that polls service availability
  // and fires the request once the service shows up. Avoids stalling the node
  // constructor for up to 2 s when the Kachaka bridge is not (yet) running.
  // The bridge's manual_control component additionally has a lazy-enable
  // fallback on the first cmd_vel, so a missing service here is non-fatal.
  enable_manual_control_pending_value_ = enable;
  enable_manual_control_timer_ = create_wall_timer(
    500ms, [this]() {
      if (!enable_manual_control_client_->service_is_ready()) {
        return;
      }
      auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
      req->data = enable_manual_control_pending_value_;
      enable_manual_control_client_->async_send_request(req);
      RCLCPP_INFO(
        get_logger(), "Requested set_manual_control_enabled(%s)",
        enable_manual_control_pending_value_ ? "true" : "false");
      enable_manual_control_timer_->cancel();
    });
}

}  // namespace kachaka_autoware_vehicle_interface

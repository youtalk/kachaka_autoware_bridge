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

#ifndef KACHAKA_AUTOWARE_VEHICLE_INTERFACE__VEHICLE_INTERFACE_NODE_HPP_
#define KACHAKA_AUTOWARE_VEHICLE_INTERFACE__VEHICLE_INTERFACE_NODE_HPP_

#include <memory>

#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_adapi_v1_msgs/srv/change_operation_mode.hpp>
#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_vehicle_msgs/msg/velocity_report.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "kachaka_autoware_vehicle_interface/control_to_twist_converter.hpp"
#include "kachaka_autoware_vehicle_interface/operation_mode_state_machine.hpp"
#include "kachaka_autoware_vehicle_interface/velocity_status_publisher.hpp"

namespace kachaka_autoware_vehicle_interface {

class VehicleInterfaceNode : public rclcpp::Node {
 public:
  explicit VehicleInterfaceNode(const rclcpp::NodeOptions& options);

 private:
  std::unique_ptr<ControlToTwistConverter> converter_;
  OperationModeStateMachine state_machine_;

  rclcpp::Subscription<autoware_control_msgs::msg::Control>::SharedPtr
      control_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::VelocityReport>::SharedPtr
      velocity_status_pub_;
  rclcpp::Publisher<autoware_adapi_v1_msgs::msg::OperationModeState>::SharedPtr
      op_mode_pub_;
  rclcpp::Service<autoware_adapi_v1_msgs::srv::ChangeOperationMode>::SharedPtr
      change_to_autonomous_srv_;
  rclcpp::Service<autoware_adapi_v1_msgs::srv::ChangeOperationMode>::SharedPtr
      change_to_stop_srv_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr
      enable_manual_control_client_;

  rclcpp::TimerBase::SharedPtr velocity_status_timer_;
  rclcpp::TimerBase::SharedPtr op_mode_timer_;
  rclcpp::TimerBase::SharedPtr cmd_vel_timeout_timer_;
  rclcpp::TimerBase::SharedPtr enable_manual_control_timer_;

  nav_msgs::msg::Odometry::SharedPtr latest_odom_;
  rclcpp::Time last_control_stamp_;
  double cmd_vel_timeout_sec_;
  bool enable_manual_control_pending_value_{false};

  void on_control(const autoware_control_msgs::msg::Control::SharedPtr msg);
  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void on_velocity_status_timer();
  void on_op_mode_timer();
  void on_cmd_vel_timeout_timer();
  void on_change_to_autonomous(
      const autoware_adapi_v1_msgs::srv::ChangeOperationMode::Request::SharedPtr
          req,
      autoware_adapi_v1_msgs::srv::ChangeOperationMode::Response::SharedPtr
          resp);
  void on_change_to_stop(
      const autoware_adapi_v1_msgs::srv::ChangeOperationMode::Request::SharedPtr
          req,
      autoware_adapi_v1_msgs::srv::ChangeOperationMode::Response::SharedPtr
          resp);

  void enable_manual_control(bool enable);
};

}  // namespace kachaka_autoware_vehicle_interface

#endif  // KACHAKA_AUTOWARE_VEHICLE_INTERFACE__VEHICLE_INTERFACE_NODE_HPP_

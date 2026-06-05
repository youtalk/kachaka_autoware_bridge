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

#ifndef KACHAKA_AUTOWARE_LOCALIZATION__LOCALIZATION_BRIDGE_NODE_HPP_
#define KACHAKA_AUTOWARE_LOCALIZATION__LOCALIZATION_BRIDGE_NODE_HPP_

#include <memory>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "kachaka_autoware_localization/kinematic_state_converter.hpp"

namespace kachaka_autoware_localization {

class LocalizationBridgeNode : public rclcpp::Node {
 public:
  explicit LocalizationBridgeNode(const rclcpp::NodeOptions& options);

 private:
  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg);

  KinematicStateParams params_;
  double tf_timeout_sec_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr kinematic_state_pub_;
};

}  // namespace kachaka_autoware_localization

#endif  // KACHAKA_AUTOWARE_LOCALIZATION__LOCALIZATION_BRIDGE_NODE_HPP_

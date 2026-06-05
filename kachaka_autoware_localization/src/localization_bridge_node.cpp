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

#include "kachaka_autoware_localization/localization_bridge_node.hpp"

#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>

namespace kachaka_autoware_localization {

LocalizationBridgeNode::LocalizationBridgeNode(
    const rclcpp::NodeOptions& options)
    : rclcpp::Node("kachaka_autoware_localization", options) {
  params_.map_frame = declare_parameter<std::string>("map_frame", "map");
  params_.base_frame =
      declare_parameter<std::string>("base_frame", "base_link");
  params_.pose_covariance_xy =
      declare_parameter<double>("pose_covariance_xy", 0.01);
  params_.pose_covariance_yaw =
      declare_parameter<double>("pose_covariance_yaw", 0.01);
  params_.twist_covariance_vx =
      declare_parameter<double>("twist_covariance_vx", 0.01);
  params_.twist_covariance_wz =
      declare_parameter<double>("twist_covariance_wz", 0.01);
  tf_timeout_sec_ = declare_parameter<double>("tf_timeout", 0.2);
  const std::string odom_topic = declare_parameter<std::string>(
      "odometry_topic", "/kachaka/odometry/odometry");

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  // The single-argument TransformListener spins /tf and /tf_static on its own
  // dedicated thread (spin_thread=true). This is intentional and required: the
  // blocking lookupTransform(..., durationFromSec(tf_timeout_sec_)) in on_odom
  // runs on the main spin thread, so the buffer must be filled by a separate
  // thread for that bounded wait to ever succeed.
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // SensorDataQoS (BEST_EFFORT) matches Kachaka's odometry publisher; the
  // kinematic_state publisher below stays RELIABLE for Autoware consumers.
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::SensorDataQoS(),
      std::bind(&LocalizationBridgeNode::on_odom, this, std::placeholders::_1));
  kinematic_state_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      "/localization/kinematic_state", rclcpp::QoS(1));

  RCLCPP_INFO(get_logger(),
              "kachaka_autoware_localization started: %s -> "
              "/localization/kinematic_state (%s -> %s)",
              odom_topic.c_str(), params_.map_frame.c_str(),
              params_.base_frame.c_str());
}

void LocalizationBridgeNode::on_odom(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_->lookupTransform(params_.map_frame, params_.base_frame,
                                     tf2::TimePointZero,
                                     tf2::durationFromSec(tf_timeout_sec_));
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "%s->%s TF unavailable: %s", params_.map_frame.c_str(),
                         params_.base_frame.c_str(), ex.what());
    return;
  }

  // Pose from the latest map->base_link TF; twist from Kachaka odometry. The
  // odometry child frame is base_footprint, but base_footprint->base_link is a
  // fixed Z offset with no planar rotation, so (vx, wz) carry over unchanged.
  // The output is stamped with the odometry time while the pose is the latest
  // available TF (TimePointZero); at ~10 Hz the pose age is ~one TF cycle,
  // acceptable for M1.
  kinematic_state_pub_->publish(ToKinematicState(
      msg->header.stamp, tf.transform, msg->twist.twist, params_));
}

}  // namespace kachaka_autoware_localization

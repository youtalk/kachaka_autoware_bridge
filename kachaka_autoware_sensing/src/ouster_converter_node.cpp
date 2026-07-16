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

#include "kachaka_autoware_sensing/ouster_converter_node.hpp"

#include <functional>
#include <string>

#include "kachaka_autoware_sensing/pointcloud_converter.hpp"

namespace kachaka_autoware_sensing {

OusterConverterNode::OusterConverterNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("ouster_converter_node", options) {
  const auto input_topic =
      declare_parameter<std::string>("input_topic", "/ouster/points");
  const auto output_topic = declare_parameter<std::string>(
      "output_topic", "/sensing/lidar/top/pointcloud_raw_ex");

  pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic, rclcpp::SensorDataQoS());
  sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic, rclcpp::SensorDataQoS(),
      std::bind(&OusterConverterNode::on_cloud, this, std::placeholders::_1));
}

void OusterConverterNode::on_cloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  if (!HasOusterFields(*msg)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "Input cloud is missing required Ouster fields "
                         "(x, y, z, intensity, ring); dropping.");
    return;
  }
  pub_->publish(ToPointXYZIRC(*msg));
}

}  // namespace kachaka_autoware_sensing

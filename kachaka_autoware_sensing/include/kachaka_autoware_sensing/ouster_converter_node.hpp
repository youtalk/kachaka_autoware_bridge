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

#ifndef KACHAKA_AUTOWARE_SENSING__OUSTER_CONVERTER_NODE_HPP_
#define KACHAKA_AUTOWARE_SENSING__OUSTER_CONVERTER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace kachaka_autoware_sensing {

// Subscribes to a raw Ouster cloud and republishes it in Autoware's
// PointXYZIRC layout. Clouds that lack the required Ouster fields are dropped
// with a throttled warning.
class OusterConverterNode : public rclcpp::Node {
 public:
  explicit OusterConverterNode(const rclcpp::NodeOptions& options);

 private:
  void on_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

}  // namespace kachaka_autoware_sensing

#endif  // KACHAKA_AUTOWARE_SENSING__OUSTER_CONVERTER_NODE_HPP_

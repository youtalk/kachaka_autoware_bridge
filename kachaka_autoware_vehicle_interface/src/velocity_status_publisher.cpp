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

#include "kachaka_autoware_vehicle_interface/velocity_status_publisher.hpp"

namespace kachaka_autoware_vehicle_interface
{

autoware_vehicle_msgs::msg::VelocityReport convert_odometry_to_velocity_report(
  const nav_msgs::msg::Odometry & odom)
{
  autoware_vehicle_msgs::msg::VelocityReport report;
  // VelocityReport carries body-frame velocities, so the frame_id must match
  // the frame Odometry's twist is expressed in (child_frame_id, e.g.
  // base_link), not the pose frame (header.frame_id, e.g. odom/map).
  report.header.stamp = odom.header.stamp;
  report.header.frame_id = odom.child_frame_id;
  report.longitudinal_velocity = static_cast<float>(odom.twist.twist.linear.x);
  // Differential drive: lateral velocity in body frame is identically zero.
  report.lateral_velocity = 0.0f;
  report.heading_rate = static_cast<float>(odom.twist.twist.angular.z);
  return report;
}

}  // namespace kachaka_autoware_vehicle_interface

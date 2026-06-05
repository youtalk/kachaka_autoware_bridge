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

#include "kachaka_autoware_localization/kinematic_state_converter.hpp"

namespace kachaka_autoware_localization {

namespace {
// Fixed small variance for axes a planar differential-drive robot does not move
// along (z, roll, pitch, vy, vz, wx, wy). Non-zero keeps downstream covariance
// inversion well-conditioned.
constexpr double kFixedAxisVariance = 1e-4;
}  // namespace

nav_msgs::msg::Odometry ToKinematicState(
    const builtin_interfaces::msg::Time& stamp,
    const geometry_msgs::msg::Transform& map_to_base,
    const geometry_msgs::msg::Twist& body_twist,
    const KinematicStateParams& params) {
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = params.map_frame;
  odom.child_frame_id = params.base_frame;

  odom.pose.pose.position.x = map_to_base.translation.x;
  odom.pose.pose.position.y = map_to_base.translation.y;
  odom.pose.pose.position.z = map_to_base.translation.z;
  odom.pose.pose.orientation = map_to_base.rotation;

  // 6x6 row-major covariance, diagonal [x, y, z, roll, pitch, yaw].
  odom.pose.covariance[0] = params.pose_covariance_xy;    // x
  odom.pose.covariance[7] = params.pose_covariance_xy;    // y
  odom.pose.covariance[14] = kFixedAxisVariance;          // z
  odom.pose.covariance[21] = kFixedAxisVariance;          // roll
  odom.pose.covariance[28] = kFixedAxisVariance;          // pitch
  odom.pose.covariance[35] = params.pose_covariance_yaw;  // yaw

  odom.twist.twist = body_twist;
  odom.twist.covariance[0] = params.twist_covariance_vx;   // vx
  odom.twist.covariance[7] = kFixedAxisVariance;           // vy
  odom.twist.covariance[14] = kFixedAxisVariance;          // vz
  odom.twist.covariance[21] = kFixedAxisVariance;          // wx
  odom.twist.covariance[28] = kFixedAxisVariance;          // wy
  odom.twist.covariance[35] = params.twist_covariance_wz;  // wz

  return odom;
}

}  // namespace kachaka_autoware_localization

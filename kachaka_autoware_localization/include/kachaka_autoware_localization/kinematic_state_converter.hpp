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

#ifndef KACHAKA_AUTOWARE_LOCALIZATION__KINEMATIC_STATE_CONVERTER_HPP_
#define KACHAKA_AUTOWARE_LOCALIZATION__KINEMATIC_STATE_CONVERTER_HPP_

#include <string>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace kachaka_autoware_localization {

struct KinematicStateParams {
  std::string map_frame;
  std::string base_frame;
  double pose_covariance_xy;
  double pose_covariance_yaw;
  double twist_covariance_vx;
  double twist_covariance_wz;
};

// Build an Autoware /localization/kinematic_state (nav_msgs/Odometry in the map
// frame) from a map->base_link TF transform (pose) and a body-frame twist taken
// from Kachaka's odometry. Pose/twist covariances are filled on the diagonal
// from params; unused axes get a small fixed variance so downstream covariance
// inversion stays well-conditioned.
nav_msgs::msg::Odometry ToKinematicState(
    const builtin_interfaces::msg::Time& stamp,
    const geometry_msgs::msg::Transform& map_to_base,
    const geometry_msgs::msg::Twist& body_twist,
    const KinematicStateParams& params);

}  // namespace kachaka_autoware_localization

#endif  // KACHAKA_AUTOWARE_LOCALIZATION__KINEMATIC_STATE_CONVERTER_HPP_

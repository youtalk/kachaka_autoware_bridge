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

#ifndef KACHAKA_AUTOWARE_VEHICLE_INTERFACE__VELOCITY_STATUS_PUBLISHER_HPP_
#define KACHAKA_AUTOWARE_VEHICLE_INTERFACE__VELOCITY_STATUS_PUBLISHER_HPP_

#include <autoware_vehicle_msgs/msg/velocity_report.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace kachaka_autoware_vehicle_interface {

autoware_vehicle_msgs::msg::VelocityReport convert_odometry_to_velocity_report(
    const nav_msgs::msg::Odometry& odom);

}  // namespace kachaka_autoware_vehicle_interface

#endif  // KACHAKA_AUTOWARE_VEHICLE_INTERFACE__VELOCITY_STATUS_PUBLISHER_HPP_

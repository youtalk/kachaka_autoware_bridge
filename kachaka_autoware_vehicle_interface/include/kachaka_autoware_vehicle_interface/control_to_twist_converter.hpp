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

#ifndef KACHAKA_AUTOWARE_VEHICLE_INTERFACE__CONTROL_TO_TWIST_CONVERTER_HPP_
#define KACHAKA_AUTOWARE_VEHICLE_INTERFACE__CONTROL_TO_TWIST_CONVERTER_HPP_

#include <autoware_control_msgs/msg/control.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace kachaka_autoware_vehicle_interface
{

struct ControlToTwistParams
{
  double wheel_base;
  double max_linear_velocity;
  double max_angular_velocity;
};

class ControlToTwistConverter
{
public:
  explicit ControlToTwistConverter(const ControlToTwistParams & params);

  geometry_msgs::msg::Twist convert(const autoware_control_msgs::msg::Control & control) const;

private:
  ControlToTwistParams params_;
};

}  // namespace kachaka_autoware_vehicle_interface

#endif  // KACHAKA_AUTOWARE_VEHICLE_INTERFACE__CONTROL_TO_TWIST_CONVERTER_HPP_

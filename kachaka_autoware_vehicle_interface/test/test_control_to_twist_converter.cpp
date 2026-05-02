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

#include <gtest/gtest.h>

#include <cmath>

#include <autoware_control_msgs/msg/control.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "kachaka_autoware_vehicle_interface/control_to_twist_converter.hpp"

using kachaka_autoware_vehicle_interface::ControlToTwistConverter;
using kachaka_autoware_vehicle_interface::ControlToTwistParams;

namespace
{

ControlToTwistParams make_default_params()
{
  ControlToTwistParams p;
  p.wheel_base = 0.30;
  p.max_linear_velocity = 0.3;
  p.max_angular_velocity = 1.57;
  return p;
}

autoware_control_msgs::msg::Control make_control(double v, double delta)
{
  autoware_control_msgs::msg::Control c;
  c.longitudinal.velocity = static_cast<float>(v);
  c.lateral.steering_tire_angle = static_cast<float>(delta);
  return c;
}

}  // namespace

TEST(ControlToTwistConverter, StraightLineHasZeroAngular)
{
  ControlToTwistConverter converter(make_default_params());
  const auto twist = converter.convert(make_control(0.2, 0.0));
  EXPECT_DOUBLE_EQ(twist.linear.x, static_cast<double>(static_cast<float>(0.2)));
  EXPECT_DOUBLE_EQ(twist.angular.z, 0.0);
}

TEST(ControlToTwistConverter, RightTurnHasNegativeAngular)
{
  // delta = -0.3 rad (right turn), v = 0.2 m/s, wheel_base = 0.30
  // omega = 0.2 * tan(-0.3) / 0.30
  ControlToTwistConverter converter(make_default_params());
  const auto twist = converter.convert(make_control(0.2, -0.3));
  const double v_f = static_cast<double>(static_cast<float>(0.2));
  const double delta_f = static_cast<double>(static_cast<float>(-0.3));
  EXPECT_NEAR(twist.linear.x, v_f, 1e-6);
  EXPECT_NEAR(twist.angular.z, v_f * std::tan(delta_f) / 0.30, 1e-6);
  EXPECT_LT(twist.angular.z, 0.0);
}

TEST(ControlToTwistConverter, ClampLinearVelocityToMax)
{
  ControlToTwistConverter converter(make_default_params());
  const auto twist = converter.convert(make_control(1.0, 0.0));
  EXPECT_NEAR(twist.linear.x, 0.3, 1e-9);
}

TEST(ControlToTwistConverter, ClampAngularVelocityToMax)
{
  ControlToTwistConverter converter(make_default_params());
  // Large delta yields a huge omega; clamped to max_angular_velocity.
  const auto twist = converter.convert(make_control(0.3, 1.5));
  EXPECT_NEAR(twist.angular.z, 1.57, 1e-6);
}

TEST(ControlToTwistConverter, NegativeLinearVelocityClampsAtNegativeMax)
{
  ControlToTwistConverter converter(make_default_params());
  const auto twist = converter.convert(make_control(-1.0, 0.0));
  EXPECT_NEAR(twist.linear.x, -0.3, 1e-9);
}

TEST(ControlToTwistConverter, ZeroWheelBaseGivesZeroAngular)
{
  // Defensive: wheel_base = 0 must not divide by zero.
  ControlToTwistParams p;
  p.wheel_base = 0.0;
  p.max_linear_velocity = 0.3;
  p.max_angular_velocity = 1.57;
  ControlToTwistConverter converter(p);
  const auto twist = converter.convert(make_control(0.2, 0.5));
  EXPECT_DOUBLE_EQ(twist.angular.z, 0.0);
}

TEST(ControlToTwistConverter, ZeroVelocityZeroSteerYieldsZeroTwist)
{
  ControlToTwistConverter converter(make_default_params());
  const auto twist = converter.convert(make_control(0.0, 0.0));
  EXPECT_DOUBLE_EQ(twist.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(twist.angular.z, 0.0);
}

TEST(ControlToTwistConverter, LinearSaturationPreservesCurvature)
{
  // Requested v = 1.0 m/s gets clamped to 0.3, but the steering angle is
  // small enough that omega remains well within saturation. The resulting
  // (v_clamped, omega) pair must still describe the same curvature
  // tan(delta)/L the controller asked for; i.e. omega = v_clamped * tan(delta) / L,
  // not v_unclamped * tan(delta) / L. Computing omega from the unclamped v
  // (the previous behavior) would over-rotate by the saturation ratio
  // 1.0/0.3 ≈ 3.3x.
  ControlToTwistConverter converter(make_default_params());
  const double delta = 0.1;
  const auto twist = converter.convert(make_control(1.0, delta));

  const double delta_f = static_cast<double>(static_cast<float>(delta));
  const double expected_omega = 0.3 * std::tan(delta_f) / 0.30;
  EXPECT_NEAR(twist.linear.x, 0.3, 1e-9);
  EXPECT_NEAR(twist.angular.z, expected_omega, 1e-6);
  // Sanity check against the would-be buggy value.
  const double buggy_omega = 1.0 * std::tan(delta_f) / 0.30;
  EXPECT_LT(std::abs(twist.angular.z), std::abs(buggy_omega));
}

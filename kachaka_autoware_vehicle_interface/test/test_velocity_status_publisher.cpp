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
#include <nav_msgs/msg/odometry.hpp>

#include "kachaka_autoware_vehicle_interface/velocity_status_publisher.hpp"

using kachaka_autoware_vehicle_interface::convert_odometry_to_velocity_report;

TEST(VelocityStatusPublisher, ConvertsLinearAndAngularComponents) {
  nav_msgs::msg::Odometry odom;
  odom.header.stamp.sec = 42;
  odom.header.stamp.nanosec = 123;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";
  odom.twist.twist.linear.x = 0.15;
  odom.twist.twist.angular.z = -0.5;

  const auto report = convert_odometry_to_velocity_report(odom);
  EXPECT_EQ(report.header.stamp.sec, 42);
  EXPECT_EQ(report.header.stamp.nanosec, 123u);
  // VelocityReport must use the body frame the twist is expressed in, not
  // the pose's reference frame.
  EXPECT_EQ(report.header.frame_id, "base_link");
  EXPECT_FLOAT_EQ(report.longitudinal_velocity, 0.15f);
  EXPECT_FLOAT_EQ(report.lateral_velocity, 0.0f);
  EXPECT_FLOAT_EQ(report.heading_rate, -0.5f);
}

TEST(VelocityStatusPublisher, FrameIdComesFromChildFrameNotPoseFrame) {
  // Regression guard: nav_msgs/Odometry.twist is defined in child_frame_id.
  // Using header.frame_id here would mis-label body-frame velocities as the
  // world frame whenever the source odometry is in odom/map.
  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";
  const auto report = convert_odometry_to_velocity_report(odom);
  EXPECT_NE(report.header.frame_id, "odom");
  EXPECT_EQ(report.header.frame_id, "base_link");
}

TEST(VelocityStatusPublisher, IgnoresLateralLinearVelocity) {
  // Even if the source odometry carries a non-zero linear.y (which would be
  // physically meaningless for a differential-drive base), the converter must
  // emit lateral_velocity = 0.
  nav_msgs::msg::Odometry odom;
  odom.twist.twist.linear.x = 0.1;
  odom.twist.twist.linear.y = 99.0;
  odom.twist.twist.angular.z = 0.0;

  const auto report = convert_odometry_to_velocity_report(odom);
  EXPECT_FLOAT_EQ(report.longitudinal_velocity, 0.1f);
  EXPECT_FLOAT_EQ(report.lateral_velocity, 0.0f);
}

TEST(VelocityStatusPublisher, ZeroOdometryYieldsZeroReport) {
  nav_msgs::msg::Odometry odom;
  const auto report = convert_odometry_to_velocity_report(odom);
  EXPECT_FLOAT_EQ(report.longitudinal_velocity, 0.0f);
  EXPECT_FLOAT_EQ(report.lateral_velocity, 0.0f);
  EXPECT_FLOAT_EQ(report.heading_rate, 0.0f);
}

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

#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "kachaka_autoware_localization/kinematic_state_converter.hpp"

using kachaka_autoware_localization::KinematicStateParams;
using kachaka_autoware_localization::ToKinematicState;

namespace {

KinematicStateParams make_params() {
  KinematicStateParams p;
  p.map_frame = "map";
  p.base_frame = "base_link";
  p.pose_covariance_xy = 0.01;
  p.pose_covariance_yaw = 0.02;
  p.twist_covariance_vx = 0.03;
  p.twist_covariance_wz = 0.04;
  return p;
}

}  // namespace

TEST(KinematicStateConverter, SetsFramesAndStamp) {
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 123;
  stamp.nanosec = 456;
  const auto odom = ToKinematicState(stamp, geometry_msgs::msg::Transform(),
                                     geometry_msgs::msg::Twist(), make_params());
  EXPECT_EQ(odom.header.frame_id, "map");
  EXPECT_EQ(odom.child_frame_id, "base_link");
  EXPECT_EQ(odom.header.stamp.sec, 123);
  EXPECT_EQ(odom.header.stamp.nanosec, 456u);
}

TEST(KinematicStateConverter, CopiesPoseFromTransform) {
  geometry_msgs::msg::Transform tf;
  tf.translation.x = 1.5;
  tf.translation.y = -2.5;
  tf.translation.z = 0.0;
  tf.rotation.z = 0.7071068;
  tf.rotation.w = 0.7071068;
  const auto odom = ToKinematicState(builtin_interfaces::msg::Time(), tf,
                                     geometry_msgs::msg::Twist(), make_params());
  EXPECT_DOUBLE_EQ(odom.pose.pose.position.x, 1.5);
  EXPECT_DOUBLE_EQ(odom.pose.pose.position.y, -2.5);
  EXPECT_DOUBLE_EQ(odom.pose.pose.position.z, 0.0);
  EXPECT_DOUBLE_EQ(odom.pose.pose.orientation.x, 0.0);
  EXPECT_DOUBLE_EQ(odom.pose.pose.orientation.y, 0.0);
  EXPECT_DOUBLE_EQ(odom.pose.pose.orientation.z, 0.7071068);
  EXPECT_DOUBLE_EQ(odom.pose.pose.orientation.w, 0.7071068);
}

TEST(KinematicStateConverter, CopiesTwist) {
  geometry_msgs::msg::Twist twist;
  twist.linear.x = 0.2;
  twist.angular.z = -0.1;
  const auto odom =
      ToKinematicState(builtin_interfaces::msg::Time(),
                       geometry_msgs::msg::Transform(), twist, make_params());
  EXPECT_DOUBLE_EQ(odom.twist.twist.linear.x, 0.2);
  EXPECT_DOUBLE_EQ(odom.twist.twist.linear.y, 0.0);
  EXPECT_DOUBLE_EQ(odom.twist.twist.angular.z, -0.1);
}

TEST(KinematicStateConverter, FillsCovarianceDiagonal) {
  const auto odom =
      ToKinematicState(builtin_interfaces::msg::Time(),
                       geometry_msgs::msg::Transform(),
                       geometry_msgs::msg::Twist(), make_params());
  EXPECT_DOUBLE_EQ(odom.pose.covariance[0], 0.01);    // x
  EXPECT_DOUBLE_EQ(odom.pose.covariance[7], 0.01);    // y
  EXPECT_DOUBLE_EQ(odom.pose.covariance[35], 0.02);   // yaw
  EXPECT_DOUBLE_EQ(odom.twist.covariance[0], 0.03);   // vx
  EXPECT_DOUBLE_EQ(odom.twist.covariance[35], 0.04);  // wz
  constexpr double kFixed = 1e-4;
  EXPECT_DOUBLE_EQ(odom.pose.covariance[14], kFixed);   // z
  EXPECT_DOUBLE_EQ(odom.pose.covariance[21], kFixed);   // roll
  EXPECT_DOUBLE_EQ(odom.pose.covariance[28], kFixed);   // pitch
  EXPECT_DOUBLE_EQ(odom.twist.covariance[7], kFixed);   // vy
  EXPECT_DOUBLE_EQ(odom.twist.covariance[14], kFixed);  // vz
  EXPECT_DOUBLE_EQ(odom.twist.covariance[21], kFixed);  // wx
  EXPECT_DOUBLE_EQ(odom.twist.covariance[28], kFixed);  // wy
  EXPECT_DOUBLE_EQ(odom.pose.covariance[1], 0.0);   // off-diagonal stays zero
  EXPECT_DOUBLE_EQ(odom.twist.covariance[1], 0.0);  // off-diagonal stays zero
}

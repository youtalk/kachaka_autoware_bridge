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

#include <cstddef>
#include <cstdint>

#include <gtest/gtest.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "kachaka_autoware_sensing/pointcloud_converter.hpp"

namespace {

// Build a minimal native-Ouster-style cloud (x, y, z, intensity float32; ring
// uint16) with two points. Field offsets differ from the real sensor, but the
// converter reads by field name, so the exact offsets do not matter here.
sensor_msgs::msg::PointCloud2 MakeOusterCloud() {
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "os_lidar";
  cloud.header.stamp.sec = 123;
  cloud.header.stamp.nanosec = 456U;

  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2Fields(
      5, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
      sensor_msgs::msg::PointField::FLOAT32, "z", 1,
      sensor_msgs::msg::PointField::FLOAT32, "intensity", 1,
      sensor_msgs::msg::PointField::FLOAT32, "ring", 1,
      sensor_msgs::msg::PointField::UINT16);
  modifier.resize(2);

  sensor_msgs::PointCloud2Iterator<float> it_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> it_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> it_z(cloud, "z");
  sensor_msgs::PointCloud2Iterator<float> it_intensity(cloud, "intensity");
  sensor_msgs::PointCloud2Iterator<std::uint16_t> it_ring(cloud, "ring");

  // Point 0: intensity 1000 > 255 must clamp to 255.
  *it_x = 1.0F;
  *it_y = 2.0F;
  *it_z = 3.0F;
  *it_intensity = 1000.0F;
  *it_ring = 7U;
  ++it_x;
  ++it_y;
  ++it_z;
  ++it_intensity;
  ++it_ring;
  // Point 1.
  *it_x = -4.0F;
  *it_y = 5.5F;
  *it_z = -6.0F;
  *it_intensity = 42.0F;
  *it_ring = 63U;

  return cloud;
}

}  // namespace

TEST(PointcloudConverter, EmitsExactPointXYZIRCLayout) {
  const auto out = kachaka_autoware_sensing::ToPointXYZIRC(MakeOusterCloud());

  ASSERT_EQ(out.fields.size(), 6U);
  EXPECT_EQ(out.point_step, 16U);

  EXPECT_EQ(out.fields[0].name, "x");
  EXPECT_EQ(out.fields[0].offset, 0U);
  EXPECT_EQ(out.fields[0].datatype, sensor_msgs::msg::PointField::FLOAT32);
  EXPECT_EQ(out.fields[1].name, "y");
  EXPECT_EQ(out.fields[1].offset, 4U);
  EXPECT_EQ(out.fields[1].datatype, sensor_msgs::msg::PointField::FLOAT32);
  EXPECT_EQ(out.fields[2].name, "z");
  EXPECT_EQ(out.fields[2].offset, 8U);
  EXPECT_EQ(out.fields[2].datatype, sensor_msgs::msg::PointField::FLOAT32);
  EXPECT_EQ(out.fields[3].name, "intensity");
  EXPECT_EQ(out.fields[3].offset, 12U);
  EXPECT_EQ(out.fields[3].datatype, sensor_msgs::msg::PointField::UINT8);
  EXPECT_EQ(out.fields[4].name, "return_type");
  EXPECT_EQ(out.fields[4].offset, 13U);
  EXPECT_EQ(out.fields[4].datatype, sensor_msgs::msg::PointField::UINT8);
  EXPECT_EQ(out.fields[5].name, "channel");
  EXPECT_EQ(out.fields[5].offset, 14U);
  EXPECT_EQ(out.fields[5].datatype, sensor_msgs::msg::PointField::UINT16);
}

TEST(PointcloudConverter, CopiesValuesAndClampsIntensity) {
  const auto out = kachaka_autoware_sensing::ToPointXYZIRC(MakeOusterCloud());

  ASSERT_EQ(static_cast<std::size_t>(out.width) * out.height, 2U);

  sensor_msgs::PointCloud2ConstIterator<float> it_x(out, "x");
  sensor_msgs::PointCloud2ConstIterator<float> it_y(out, "y");
  sensor_msgs::PointCloud2ConstIterator<float> it_z(out, "z");
  sensor_msgs::PointCloud2ConstIterator<std::uint8_t> it_intensity(out,
                                                                   "intensity");
  sensor_msgs::PointCloud2ConstIterator<std::uint8_t> it_return(out,
                                                                "return_type");
  sensor_msgs::PointCloud2ConstIterator<std::uint16_t> it_channel(out,
                                                                  "channel");

  EXPECT_FLOAT_EQ(*it_x, 1.0F);
  EXPECT_FLOAT_EQ(*it_y, 2.0F);
  EXPECT_FLOAT_EQ(*it_z, 3.0F);
  EXPECT_EQ(*it_intensity, 255U);  // clamped from 1000
  EXPECT_EQ(*it_return, 1U);
  EXPECT_EQ(*it_channel, 7U);

  ++it_x;
  ++it_y;
  ++it_z;
  ++it_intensity;
  ++it_return;
  ++it_channel;

  EXPECT_FLOAT_EQ(*it_x, -4.0F);
  EXPECT_FLOAT_EQ(*it_y, 5.5F);
  EXPECT_FLOAT_EQ(*it_z, -6.0F);
  EXPECT_EQ(*it_intensity, 42U);
  EXPECT_EQ(*it_return, 1U);
  EXPECT_EQ(*it_channel, 63U);
}

TEST(PointcloudConverter, PreservesHeader) {
  const auto out = kachaka_autoware_sensing::ToPointXYZIRC(MakeOusterCloud());

  EXPECT_EQ(out.header.frame_id, "os_lidar");
  EXPECT_EQ(out.header.stamp.sec, 123);
  EXPECT_EQ(out.header.stamp.nanosec, 456U);
}

TEST(PointcloudConverter, HandlesEmptyCloud) {
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "os_lidar";
  cloud.header.stamp.sec = 123;
  cloud.header.stamp.nanosec = 456U;

  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2Fields(
      5, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
      sensor_msgs::msg::PointField::FLOAT32, "z", 1,
      sensor_msgs::msg::PointField::FLOAT32, "intensity", 1,
      sensor_msgs::msg::PointField::FLOAT32, "ring", 1,
      sensor_msgs::msg::PointField::UINT16);
  modifier.resize(0);

  const auto out = kachaka_autoware_sensing::ToPointXYZIRC(cloud);

  EXPECT_EQ(static_cast<std::size_t>(out.width) * out.height, 0U);
  ASSERT_EQ(out.fields.size(), 6U);
  EXPECT_EQ(out.point_step, 16U);
  EXPECT_EQ(out.header.frame_id, "os_lidar");
  EXPECT_EQ(out.header.stamp.sec, 123);
  EXPECT_EQ(out.header.stamp.nanosec, 456U);
}

TEST(HasOusterFields, TrueWhenAllPresent) {
  EXPECT_TRUE(kachaka_autoware_sensing::HasOusterFields(MakeOusterCloud()));
}

TEST(HasOusterFields, FalseWhenRingMissing) {
  sensor_msgs::msg::PointCloud2 cloud;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2Fields(
      4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
      sensor_msgs::msg::PointField::FLOAT32, "z", 1,
      sensor_msgs::msg::PointField::FLOAT32, "intensity", 1,
      sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(1);

  EXPECT_FALSE(kachaka_autoware_sensing::HasOusterFields(cloud));
}

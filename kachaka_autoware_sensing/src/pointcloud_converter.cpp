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

#include "kachaka_autoware_sensing/pointcloud_converter.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>

#include <autoware/point_types/types.hpp>
#include <point_cloud_msg_wrapper/point_cloud_msg_wrapper.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace kachaka_autoware_sensing {

namespace {

bool HasField(const sensor_msgs::msg::PointCloud2& in,
              const std::string& name) {
  for (const auto& field : in.fields) {
    if (field.name == name) {
      return true;
    }
  }
  return false;
}

}  // namespace

bool HasOusterFields(const sensor_msgs::msg::PointCloud2& in) {
  return HasField(in, "x") && HasField(in, "y") && HasField(in, "z") &&
         HasField(in, "intensity") && HasField(in, "ring");
}

sensor_msgs::msg::PointCloud2 ToPointXYZIRC(
    const sensor_msgs::msg::PointCloud2& in) {
  using autoware::point_types::PointXYZIRC;

  sensor_msgs::msg::PointCloud2 out;
  point_cloud_msg_wrapper::PointCloud2Modifier<
      PointXYZIRC, autoware::point_types::PointXYZIRCGenerator>
      out_modifier{out, in.header.frame_id};

  const std::size_t num_points =
      static_cast<std::size_t>(in.width) * static_cast<std::size_t>(in.height);
  out_modifier.reserve(num_points);

  if (in.width == 0 || in.height == 0) {
    out.header = in.header;
    out.is_dense = in.is_dense;
    return out;
  }

  sensor_msgs::PointCloud2ConstIterator<float> it_x(in, "x");
  sensor_msgs::PointCloud2ConstIterator<float> it_y(in, "y");
  sensor_msgs::PointCloud2ConstIterator<float> it_z(in, "z");
  sensor_msgs::PointCloud2ConstIterator<float> it_intensity(in, "intensity");
  sensor_msgs::PointCloud2ConstIterator<std::uint16_t> it_ring(in, "ring");

  for (; it_x != it_x.end();
       ++it_x, ++it_y, ++it_z, ++it_intensity, ++it_ring) {
    PointXYZIRC point;
    point.x = *it_x;
    point.y = *it_y;
    point.z = *it_z;
    point.intensity = static_cast<std::uint8_t>(
        std::clamp<std::int64_t>(std::lround(*it_intensity), 0, 255));
    point.return_type = 1U;  // SINGLE_STRONGEST (single-return Ouster)
    point.channel = *it_ring;
    out_modifier.push_back(point);
  }

  // The modifier only set frame_id; restore the full header (stamp) and the
  // density flag from the input.
  out.header = in.header;
  out.is_dense = in.is_dense;
  return out;
}

}  // namespace kachaka_autoware_sensing

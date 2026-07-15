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

#ifndef KACHAKA_AUTOWARE_SENSING__POINTCLOUD_CONVERTER_HPP_
#define KACHAKA_AUTOWARE_SENSING__POINTCLOUD_CONVERTER_HPP_

#include <sensor_msgs/msg/point_cloud2.hpp>

namespace kachaka_autoware_sensing {

// Returns true iff `in` carries every field the converter reads: x, y, z,
// intensity and ring. Callers should drop clouds that fail this check rather
// than pass them to ToPointXYZIRC.
bool HasOusterFields(const sensor_msgs::msg::PointCloud2& in);

// Reformat a native Ouster OS-1 cloud (x, y, z, intensity float32; ring
// uint16) into an Autoware PointXYZIRC cloud. The header (frame_id + stamp) is
// preserved verbatim so downstream TF lookups still resolve. intensity is
// clamped to [0, 255] (Ouster intensity is float32 and can exceed 255; NDT and
// the ground filter ignore intensity for geometry, so clamping is lossless for
// this use). return_type is set to 0 (single return); channel is copied from
// ring. Output is unorganized (height 1). Precondition: the input carries the
// x, y, z, intensity and ring fields.
sensor_msgs::msg::PointCloud2 ToPointXYZIRC(
    const sensor_msgs::msg::PointCloud2& in);

}  // namespace kachaka_autoware_sensing

#endif  // KACHAKA_AUTOWARE_SENSING__POINTCLOUD_CONVERTER_HPP_

# M1: Localization Bridge Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** On the stock Kachaka (no 3D LiDAR), publish Autoware's `/localization/kinematic_state` from Kachaka's built-in SLAM pose (TF `map → base_link`) + odometry twist, and bring it up with a stock launch verified on the real robot.

**Architecture:** A new `kachaka_autoware_localization` package with a pure-logic `KinematicStateConverter` (TF transform + body twist → `nav_msgs/Odometry`) and a thin `LocalizationBridgeNode` that subscribes `/kachaka/odometry/odometry`, looks up `map → base_link` from TF, and publishes `/localization/kinematic_state`. No NDT, no EKF (Kachaka already publishes `map → odom`). A `kachaka_autoware_stock.launch.xml` runs the gRPC bridge + this node.

**Tech Stack:** ROS 2 Jazzy, C++17, `ament_cmake_auto`, `tf2_ros`, gtest. Autoware Core is prebuilt at `~/ros/jazzy`. Scope: M1 only (see `docs/superpowers/specs/2026-06-04-kachaka-autoware-stock-localization.md` for M2–M4).

---

## File Structure

New package `kachaka_autoware_localization/`:

- `package.xml` — ament_cmake_auto package, deps on rclcpp/nav_msgs/geometry_msgs/tf2/tf2_ros.
- `CMakeLists.txt` — library (converter + node) + executable + gtest.
- `include/kachaka_autoware_localization/kinematic_state_converter.hpp` — pure-logic API.
- `src/kinematic_state_converter.cpp` — pure-logic impl (unit-tested).
- `include/kachaka_autoware_localization/localization_bridge_node.hpp` — node class.
- `src/localization_bridge_node.cpp` — node impl (TF lookup + publish).
- `src/main.cpp` — executable entry point.
- `config/localization.param.yaml` — frames, topic, covariances, tf timeout.
- `launch/localization.launch.xml` — runs the node with the param file.
- `test/test_kinematic_state_converter.cpp` — gtest for the converter.

Modified `kachaka_autoware_bridge/`:

- `launch/kachaka_autoware_stock.launch.xml` — new stock bring-up (bridge + localization).
- `package.xml` — add `<exec_depend>kachaka_autoware_localization</exec_depend>`.

---

## Task 1: Pure-logic KinematicStateConverter (TDD)

**Files:**

- Create: `kachaka_autoware_localization/package.xml`
- Create: `kachaka_autoware_localization/CMakeLists.txt`
- Create: `kachaka_autoware_localization/include/kachaka_autoware_localization/kinematic_state_converter.hpp`
- Create: `kachaka_autoware_localization/src/kinematic_state_converter.cpp`
- Test: `kachaka_autoware_localization/test/test_kinematic_state_converter.cpp`

- [ ] **Step 1: Create `package.xml`**

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>kachaka_autoware_localization</name>
  <version>0.0.0</version>
  <description>Localization bridge republishing Kachaka's built-in SLAM pose (map->base_link TF) as Autoware's /localization/kinematic_state, for the stock (no 3D LiDAR) configuration.</description>
  <maintainer email="yutaka.kondo@youtalk.jp">Yutaka Kondo</maintainer>
  <license>Apache License 2.0</license>
  <author email="yutaka.kondo@youtalk.jp">Yutaka Kondo</author>

  <buildtool_depend>ament_cmake_auto</buildtool_depend>

  <depend>geometry_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>rclcpp</depend>
  <depend>tf2</depend>
  <depend>tf2_ros</depend>

  <test_depend>ament_cmake_gtest</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Step 2: Create `CMakeLists.txt`** (cmake-format will reformat on commit; that is fine)

```cmake
cmake_minimum_required(VERSION 3.14)
project(kachaka_autoware_localization)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/kinematic_state_converter.cpp
  src/localization_bridge_node.cpp)

ament_auto_add_executable(${PROJECT_NAME}_node src/main.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_kinematic_state_converter
                  test/test_kinematic_state_converter.cpp)
  target_link_libraries(test_kinematic_state_converter ${PROJECT_NAME})
endif()

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR INSTALL_TO_SHARE launch config)
```

Note: `CMakeLists.txt` references `src/localization_bridge_node.cpp`, `src/main.cpp`, `launch/`, and `config/` created in later tasks. Do **not** run a full `colcon build` until Task 4; Task 1 builds only the gtest target (Step 5).

- [ ] **Step 3: Create the header `include/kachaka_autoware_localization/kinematic_state_converter.hpp`**

```cpp
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
```

- [ ] **Step 4: Write the failing test `test/test_kinematic_state_converter.cpp`**

```cpp
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
  EXPECT_GT(odom.pose.covariance[14], 0.0);           // z variance non-zero
}
```

- [ ] **Step 5: Build the test and verify it FAILS (link error: `ToKinematicState` undefined)**

```bash
cd /home/youtalk/src/kachaka_autoware_bridge
source /opt/ros/jazzy/setup.bash
source /home/youtalk/ros/jazzy/install/setup.bash
colcon build --packages-select kachaka_autoware_localization --cmake-args -DBUILD_TESTING=ON
```

Expected: build FAILS — `undefined reference to ...ToKinematicState(...)` (the .cpp does not exist yet). This confirms the test links against the missing symbol.

- [ ] **Step 6: Implement `src/kinematic_state_converter.cpp`**

```cpp
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
```

The library target also lists `src/localization_bridge_node.cpp` (Task 2). Create a temporary empty stub so Task 1 builds in isolation, or proceed straight to Task 2 before building. Recommended: create the node files (Task 2) before the first full build. To run **only** the converter test now, temporarily comment out `src/localization_bridge_node.cpp` and the executable in `CMakeLists.txt`, build, then restore. Simpler path: continue to Task 2, then build once.

- [ ] **Step 7: Build + run the converter test, verify PASS**

```bash
cd /home/youtalk/src/kachaka_autoware_bridge
colcon build --packages-select kachaka_autoware_localization
colcon test --packages-select kachaka_autoware_localization --ctest-args -R test_kinematic_state_converter
colcon test-result --verbose
```

Expected: `test_kinematic_state_converter` PASSES (4 tests). (Requires Task 2 files present so the library links; if building before Task 2, see Step 6 note.)

- [ ] **Step 8: Commit**

```bash
git add kachaka_autoware_localization/package.xml \
        kachaka_autoware_localization/CMakeLists.txt \
        kachaka_autoware_localization/include kachaka_autoware_localization/src/kinematic_state_converter.cpp \
        kachaka_autoware_localization/test
git commit -m "feat(localization): KinematicStateConverter (TF+twist -> kinematic_state)"
```

---

## Task 2: LocalizationBridgeNode + executable

**Files:**

- Create: `kachaka_autoware_localization/include/kachaka_autoware_localization/localization_bridge_node.hpp`
- Create: `kachaka_autoware_localization/src/localization_bridge_node.cpp`
- Create: `kachaka_autoware_localization/src/main.cpp`

- [ ] **Step 1: Create the node header `localization_bridge_node.hpp`**

```cpp
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

#ifndef KACHAKA_AUTOWARE_LOCALIZATION__LOCALIZATION_BRIDGE_NODE_HPP_
#define KACHAKA_AUTOWARE_LOCALIZATION__LOCALIZATION_BRIDGE_NODE_HPP_

#include <memory>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "kachaka_autoware_localization/kinematic_state_converter.hpp"

namespace kachaka_autoware_localization {

class LocalizationBridgeNode : public rclcpp::Node {
 public:
  explicit LocalizationBridgeNode(const rclcpp::NodeOptions& options);

 private:
  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg);

  KinematicStateParams params_;
  double tf_timeout_sec_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr kinematic_state_pub_;
};

}  // namespace kachaka_autoware_localization

#endif  // KACHAKA_AUTOWARE_LOCALIZATION__LOCALIZATION_BRIDGE_NODE_HPP_
```

- [ ] **Step 2: Create the node impl `src/localization_bridge_node.cpp`**

```cpp
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

#include "kachaka_autoware_localization/localization_bridge_node.hpp"

#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>

namespace kachaka_autoware_localization {

LocalizationBridgeNode::LocalizationBridgeNode(
    const rclcpp::NodeOptions& options)
    : rclcpp::Node("kachaka_autoware_localization", options) {
  params_.map_frame = declare_parameter<std::string>("map_frame", "map");
  params_.base_frame =
      declare_parameter<std::string>("base_frame", "base_link");
  params_.pose_covariance_xy =
      declare_parameter<double>("pose_covariance_xy", 0.01);
  params_.pose_covariance_yaw =
      declare_parameter<double>("pose_covariance_yaw", 0.01);
  params_.twist_covariance_vx =
      declare_parameter<double>("twist_covariance_vx", 0.01);
  params_.twist_covariance_wz =
      declare_parameter<double>("twist_covariance_wz", 0.01);
  tf_timeout_sec_ = declare_parameter<double>("tf_timeout", 0.2);
  const std::string odom_topic = declare_parameter<std::string>(
      "odometry_topic", "/kachaka/odometry/odometry");

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::SensorDataQoS(),
      std::bind(&LocalizationBridgeNode::on_odom, this, std::placeholders::_1));
  kinematic_state_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      "/localization/kinematic_state", rclcpp::QoS(1));

  RCLCPP_INFO(get_logger(),
              "kachaka_autoware_localization started: %s -> "
              "/localization/kinematic_state (%s -> %s)",
              odom_topic.c_str(), params_.map_frame.c_str(),
              params_.base_frame.c_str());
}

void LocalizationBridgeNode::on_odom(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_->lookupTransform(params_.map_frame, params_.base_frame,
                                     tf2::TimePointZero,
                                     tf2::durationFromSec(tf_timeout_sec_));
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "%s->%s TF unavailable: %s", params_.map_frame.c_str(),
                         params_.base_frame.c_str(), ex.what());
    return;
  }

  // Pose from the latest map->base_link TF; twist from Kachaka odometry. The
  // odometry child frame is base_footprint, but base_footprint->base_link is a
  // fixed Z offset with no planar rotation, so (vx, wz) carry over unchanged.
  kinematic_state_pub_->publish(ToKinematicState(
      msg->header.stamp, tf.transform, msg->twist.twist, params_));
}

}  // namespace kachaka_autoware_localization
```

- [ ] **Step 3: Create `src/main.cpp`**

```cpp
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

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "kachaka_autoware_localization/localization_bridge_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(
      std::make_shared<kachaka_autoware_localization::LocalizationBridgeNode>(
          rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
```

- [ ] **Step 4: Build the package (library + node + test) and run the converter test**

```bash
cd /home/youtalk/src/kachaka_autoware_bridge
colcon build --packages-select kachaka_autoware_localization
colcon test --packages-select kachaka_autoware_localization
colcon test-result --verbose
```

Expected: build SUCCEEDS; `test_kinematic_state_converter` PASSES (4 tests, 0 failures). The executable `kachaka_autoware_localization_node` is produced.

- [ ] **Step 5: Commit**

```bash
git add kachaka_autoware_localization/include kachaka_autoware_localization/src
git commit -m "feat(localization): LocalizationBridgeNode publishing /localization/kinematic_state"
```

---

## Task 3: Params, node launch, and stock bring-up launch

**Files:**

- Create: `kachaka_autoware_localization/config/localization.param.yaml`
- Create: `kachaka_autoware_localization/launch/localization.launch.xml`
- Create: `kachaka_autoware_bridge/launch/kachaka_autoware_stock.launch.xml`
- Modify: `kachaka_autoware_bridge/package.xml` (add exec_depend)

- [ ] **Step 1: Create `config/localization.param.yaml`**

```yaml
/**:
  ros__parameters:
    map_frame: map
    base_frame: base_link
    odometry_topic: /kachaka/odometry/odometry
    tf_timeout: 0.2
    pose_covariance_xy: 0.01
    pose_covariance_yaw: 0.01
    twist_covariance_vx: 0.01
    twist_covariance_wz: 0.01
```

- [ ] **Step 2: Create `launch/localization.launch.xml`**

```xml
<?xml version="1.0"?>
<launch>
  <arg name="config_file"
       default="$(find-pkg-share kachaka_autoware_localization)/config/localization.param.yaml"/>

  <node pkg="kachaka_autoware_localization"
        exec="kachaka_autoware_localization_node"
        name="kachaka_autoware_localization"
        output="screen">
    <param from="$(var config_file)"/>
  </node>
</launch>
```

- [ ] **Step 3: Create `kachaka_autoware_bridge/launch/kachaka_autoware_stock.launch.xml`**

```xml
<?xml version="1.0"?>
<launch>
  <arg name="server_uri" default="192.168.1.101:26400"
       description="Kachaka gRPC URI (DHCP; override per session)"/>

  <!-- Kachaka gRPC bridge (also launches the Kachaka body URDF /
       robot_state_publisher, which provides base_footprint -> base_link). -->
  <include file="$(find-pkg-share kachaka_grpc_ros2_bridge)/launch/grpc_ros2_bridge.launch.xml">
    <arg name="server_uri" value="$(var server_uri)"/>
    <arg name="namespace" value="kachaka"/>
    <arg name="frame_prefix" value=""/>
  </include>

  <!-- Localization: republish Kachaka SLAM pose as /localization/kinematic_state.
       No NDT / EKF / pointcloud map: Kachaka already publishes map -> odom. -->
  <include file="$(find-pkg-share kachaka_autoware_localization)/launch/localization.launch.xml"/>
</launch>
```

- [ ] **Step 4: Add the exec dependency in `kachaka_autoware_bridge/package.xml`**

Insert after the existing `kachaka_autoware_vehicle_interface` exec_depend line (around line 20):

```xml
  <exec_depend>kachaka_autoware_localization</exec_depend>
```

- [ ] **Step 5: Build both packages, verify launch files install**

```bash
cd /home/youtalk/src/kachaka_autoware_bridge
colcon build --packages-select kachaka_autoware_localization kachaka_autoware_bridge
source install/setup.bash
test -f install/kachaka_autoware_bridge/share/kachaka_autoware_bridge/launch/kachaka_autoware_stock.launch.xml && echo OK_STOCK_LAUNCH
test -f install/kachaka_autoware_localization/share/kachaka_autoware_localization/launch/localization.launch.xml && echo OK_LOCALIZATION_LAUNCH
```

Expected: build SUCCEEDS; both `OK_STOCK_LAUNCH` and `OK_LOCALIZATION_LAUNCH` print.

- [ ] **Step 6: Commit**

```bash
git add kachaka_autoware_localization/config kachaka_autoware_localization/launch \
        kachaka_autoware_bridge/launch/kachaka_autoware_stock.launch.xml \
        kachaka_autoware_bridge/package.xml
git commit -m "feat(localization): stock bring-up launch (gRPC bridge + localization)"
```

---

## Task 4: Real-robot verification (no automated test — robot in the loop)

**Files:** none (verification only). Robot must be **off the dock** (the body LiDAR does not spin while docked, so `map → odom` may be static/degraded). Confirm the live `server_uri` (default `192.168.1.101:26400`).

- [ ] **Step 1: Launch the stock stack against the real robot**

```bash
cd /home/youtalk/src/kachaka_autoware_bridge
source /opt/ros/jazzy/setup.bash
source /home/youtalk/ros/jazzy/install/setup.bash
source install/setup.bash
ros2 launch kachaka_autoware_bridge kachaka_autoware_stock.launch.xml server_uri:=192.168.1.101:26400
```

Expected: node logs `kachaka_autoware_localization started: /kachaka/odometry/odometry -> /localization/kinematic_state (map -> base_link)`. Allow ~30–40 s for the bridge's `dynamic_tf` (last component) to start streaming `map → odom`; until then expect throttled `map->base_link TF unavailable` warnings — they must stop once TF is up.

- [ ] **Step 2: Verify `/localization/kinematic_state` matches Kachaka's TF (second shell)**

```bash
source /opt/ros/jazzy/setup.bash && source /home/youtalk/ros/jazzy/install/setup.bash
source /home/youtalk/src/kachaka_autoware_bridge/install/setup.bash
ros2 topic echo /localization/kinematic_state --once
ros2 run tf2_ros tf2_echo map base_link
```

Expected: `kinematic_state` has `header.frame_id: map`, `child_frame_id: base_link`, and `pose.pose` (position x/y, orientation) equal to the `tf2_echo map base_link` translation/rotation within noise. While stationary, `twist.twist.linear.x ≈ 0` and `angular.z ≈ 0`.

- [ ] **Step 3: Verify the twist tracks motion (drive the robot)**

Drive Kachaka with the smartphone app / teleop (any external means), then:

```bash
ros2 topic echo /localization/kinematic_state/twist/twist
```

Expected: `linear.x` and `angular.z` follow the motion (positive `linear.x` forward, sign of `angular.z` matching turn direction), returning to ~0 on stop.

- [ ] **Step 4: Visual confirmation in RViz**

```bash
rviz2
```

Add displays: TF; Map (topic `/kachaka/mapping/map`, Durability `Transient Local`); Odometry (topic `/localization/kinematic_state`), Fixed Frame `map`. Expected: the Kachaka occupancy map renders, and the kinematic_state pose arrow sits on the robot's true location and moves with it. This is the M1 done-criterion.

- [ ] **Step 5: Record the result**

Note in the PR description: TF startup latency observed, stationary pose error vs `tf2_echo`, and whether verification was on/off dock. Capture a short rosbag if anything looks off:

```bash
ros2 bag record /tf /tf_static /kachaka/odometry/odometry /kachaka/mapping/map /localization/kinematic_state
```

---

## Self-Review notes

- **Spec coverage:** Implements spec §5.1 (localization package + converter) and §5.2 (stock launch). Spec §5.3 (lanelet2) and §6 M2–M4 are out of scope for this plan by design.
- **Type consistency:** `ToKinematicState(stamp, transform, twist, params)` and `KinematicStateParams{map_frame, base_frame, pose_covariance_xy, pose_covariance_yaw, twist_covariance_vx, twist_covariance_wz}` are used identically in header, impl, test, and node.
- **Known follow-ups (not M1):** `/localization/initialization_state = INITIALIZED` for AD-API (M3); optional acceleration republish for `velocity_smoother` (M3); component (composable) packaging if co-location with the bridge container is wanted later.

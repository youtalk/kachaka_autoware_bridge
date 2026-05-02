# Kachaka × Autoware Core Integration Design

- Created: 2026-05-02
- Status: Draft (review stage, before implementation planning)
- Related: [autoware_core](https://github.com/autowarefoundation/autoware_core), [kachaka-api](https://github.com/pf-robotics/kachaka-api), [autoware_rviz_plugins](https://github.com/autowarefoundation/autoware_rviz_plugins)

## 1. Purpose

Drive Kachaka autonomously using Autoware Core packages. Instead of routing through Kachaka's built-in navigation, treat Kachaka as a robot on which Autoware's localization, planning, and control run directly. This brings the Autoware ecosystem assets (lanelet2 / NDT / mission_planner / pure_pursuit / AD-API / autoware_rviz_plugins) onto Kachaka.

## 2. Scope

### 2.1 In Scope (MVP)

The MVP is defined as: **specifying a single 2D Goal Pose from the standard Autoware RViz UI causes Kachaka to reach that target through Autoware's planning and control.** Concretely:

- Localization: Autoware NDT + EKF running on the Ouster OS-1 128
- Planning: full `autoware_core_planning` operation, assuming a lanelet2 vector_map
- Control: convert `autoware_simple_pure_pursuit` output into a differential-drive Twist for Kachaka
- AD-API: adopt `autoware_default_adapi` + `autoware_adapi_adaptors` + `autoware_rviz_plugins`
- Vehicle Interface: implemented as a new standalone package, leaving room to evolve toward a fully Autoware-compliant vehicle_interface in the future

### 2.2 Out of Scope (post-MVP phases)

- Dynamic object detection (perception stack)
- Patrol / multi-waypoint navigation
- Obstacle stopping (`motion_velocity_planner::ObstacleStopModule` is off in the MVP)
- Integration with Kachaka's dock/undock sequence
- Full replacement of Kachaka's built-in navigation (the `kachaka_command` family)
- Multi-Kachaka operation (use of `frame_prefix`)

## 3. Assumptions and Constraints

### 3.1 Physical Configuration

- A dedicated **"Autoware shelf"** is prepared, with a **Jetson Thor** and an **Ouster OS-1 128** mounted on top.
- This shelf is **permanently carried by Kachaka** during operation (the MVP does not handle docking transitions).
- The Kachaka body exposes its gRPC API on IP 192.168.1.91 (port 26400).
- Thor runs the full Autoware stack and `kachaka_grpc_ros2_bridge`.
- A development PC runs RViz2 + autoware_rviz_plugins, communicating with Thor over the same ROS_DOMAIN_ID.
- **Kachaka's 2D LiDAR is not used in this stack.** Kachaka's built-in mapping and localization are not relied upon; Autoware NDT is the sole source of localization. The Kachaka `/scan` topic is also not subscribed to.

### 3.2 Software Assumptions

- ROS 2 Jazzy (on Thor and on the development PC)
- ROS_DOMAIN_ID = 123
- Autoware Core workspace: `~/ros/jazzy`
- Kachaka SW version: `kachaka-api` 3.16 or later
- GNSS is unavailable (indoor)
- Indoor operation: `pose_initializer` runs with `gnss_enabled: false` and `yabloc_enabled: false`

### 3.3 Prerequisite Work

The following must be completed before entering the MVP (milestone M0):

1. **Build the pointcloud_map.** Map the home environment using **only the OS-1 128** (lio_sam, fast_lio, glim, etc.) and produce `pointcloud_map.pcd` + `pointcloud_map/metadata.yaml`. Kachaka's 2D LiDAR cannot assist here. During mapping, push or teleop Kachaka, or carry the OS-1 by hand.
2. **Build the lanelet2 vector_map.** Use TIER IV's Vector Map Builder to draw a minimal set of lanes over the home's traversable area, producing `lanelet2_map.osm` + `map_projector_info.yaml`. **The pointcloud_map and the lanelet2 vector_map must be generated with the same local projection origin** (see §4.3 for coordinate-frame consistency).
3. **Physically mount the Ouster OS-1 on the shelf** and obtain calibration values (the static transform from `base_footprint` to `os1_sensor`).

### 3.4 Design Decisions (agreed)

| Item | Choice | Notes |
|---|---|---|
| Scope | Full Autoware (C) | Includes localization |
| MVP | Single-point goal (B) | One RViz operation gets Kachaka to the goal |
| Map | Manual lanelet2 (A) | Vector Map Builder |
| Vehicle Interface | Standalone node (B, expandable to C) | Includes Operation Mode state machine |
| Initial pose | NDT monte carlo (D) | + autoware_rviz_plugins |
| AD-API | Full adoption (A) | default_adapi + adaptors + rviz_plugins |

## 4. System Architecture

### 4.1 Node Layout

```
┌────────────── Kachaka body (192.168.1.91) ──────────────┐
│  gRPC API :26400                                         │
│   - SetRobotVelocity (Twist input)                       │
│   - SetManualControlEnabled (true on Autoware Engage)    │
│   - GetRosOdometry / GetRosImu / GetRosLaserScan ...     │
└──────────────────────────────────────────────────────────┘
        ↑ gRPC over WiFi/Ethernet
        │
┌────── Autoware shelf (always mounted on Kachaka) ───────┐
│  Jetson Thor (Ubuntu 24.04 + ROS 2 Jazzy)                │
│   ├─ Ouster OS-1 128 (top of the shelf)                  │
│   ├─ ouster-ros driver                                   │
│   ├─ kachaka_grpc_ros2_bridge (existing)                 │
│   ├─ autoware_core_map / localization / planning / control │
│   ├─ autoware_default_adapi + autoware_adapi_adaptors    │
│   └─ kachaka_autoware_bridge / vehicle_interface (new)   │
└──────────────────────────────────────────────────────────┘
        │ DDS (ROS_DOMAIN_ID=123)
        │
┌────── Development PC (Jazzy) ────────────────────────────┐
│  RViz2 + autoware_rviz_plugins                           │
│  (InitialPoseButtonPanel / RouteTool / EngageButton, ...) │
└──────────────────────────────────────────────────────────┘
```

### 4.2 TF Tree

```
map  (NDT scan matcher)
 └─ odom  (published by autoware_ekf_localizer)
     └─ base_footprint
         └─ base_link  (Kachaka's kachaka_description)
             ├─ base_r_drive_wheel_link
             ├─ base_l_drive_wheel_link
             ├─ ... (existing)
             └─ docking_link  (existing prismatic, docking/lift mechanism)
                 └─ shelf_base_link  (new fixed, bottom-center of the OEM 3-tier shelf)
                     ├─ shelf_bottom_board / middle_board / top_board
                     ├─ shelf_fl_post / fr_post / bl_post / br_post
                     └─ shelf_top  (top surface of the shelf, payload mount anchor)
                         └─ os1_sensor  (new fixed, Ouster OS-1 128)
                             ├─ os1_imu
                             └─ os1_lidar
```

`map → odom`: published by `autoware_ekf_localizer` (existing).
`odom → base_footprint`: Kachaka's `dynamic_tf_bridge` (existing).
`base_footprint → base_link → ... → docking_link`: from Kachaka's `_kachaka.urdf.xacro` + `robot_state_publisher` (existing, unchanged). The `docking_link` frame sits at the `base_link` origin and the prismatic joint lifts it 0–0.012 m.
`docking_link → shelf_base_link`: when invoking the `shelf_3tier` macro from `kachaka_description/urdf/_shelf_3tier.urdf.xacro`, **pass `<origin xyz="0 0 0.115"/>` so the shelf's bottom rests on the top of the solenoid (cylinder center 0.1075 + half-length 0.0075)**. This makes the shelf follow the docking lift. The macro itself has been updated to accept an `*origin`.
`shelf_base_link → shelf_*`: same `_shelf_3tier.urdf.xacro` (**added by this update**; the OEM 3-tier shelf is a Kachaka accessory, so it belongs in `kachaka_description`).
`shelf_top → os1_sensor → os1_lidar/imu`: `kachaka_autoware_description/urdf/_ouster_os1.urdf.xacro` (new package).

### 4.3 Coordinate-Frame Consistency

- The pointcloud_map and the lanelet2 vector_map must be generated with **the same local projection origin** (the `origin` in Vector Map Builder's `map_projector_info.yaml` must match the world origin used by the SLAM tool that built the pointcloud_map).
- Autoware's map frame is operated independently of Kachaka's internal map frame (no alignment is required). Relative displacement from Kachaka's `/odometry` (odom-relative) is used as is, fed through `vehicle_velocity_converter` into the EKF, but Kachaka's absolute pose is not used.
- As long as the pointcloud_map and the lanelet2 map share the same origin, the map-frame poses produced by NDT remain consistent with planning.

## 5. New Package Layout

All new packages live under `ros2/` in the kachaka-api repository, riding on the upstream Kachaka release cycle.

```
ros2/
├── kachaka_description/                            # * Updated: macro for the OEM 3-tier shelf added
│   └── urdf/
│       └── _shelf_3tier.urdf.xacro                  # New: 3-tier shelf macro. Belongs here because the shelf is a Kachaka accessory.
│   (Existing files _materials / _values / _kachaka, etc. are not changed in a breaking way.)
│
├── kachaka_autoware_bridge/                        # Meta package + integrated launch
│   ├── package.xml
│   ├── CMakeLists.txt
│   └── launch/
│       └── kachaka_autoware.launch.xml              # All-in-one entry point
├── kachaka_autoware_vehicle_interface/              # * Core (C++ Vehicle Interface node)
│   ├── src/
│   │   ├── vehicle_interface_node.cpp               # Control→Twist + status + operation_mode
│   │   ├── vehicle_interface_node.hpp
│   │   ├── operation_mode_state_machine.cpp         # Lightweight mode decider
│   │   └── operation_mode_state_machine.hpp
│   ├── launch/vehicle_interface.launch.xml
│   ├── config/vehicle_interface.param.yaml
│   └── test/
│       └── test_control_to_twist.cpp
├── kachaka_autoware_description/                    # Full URDF (with OS-1 + shelf) + vehicle_info.yaml
│   ├── urdf/
│   │   ├── _ouster_os1.urdf.xacro                   # OS-1 128 macro (simple cylinder + lidar/imu frames)
│   │   └── kachaka_with_shelf.urdf.xacro            # Full URDF: kachaka + 3-tier shelf + OS-1
│   ├── config/
│   │   └── vehicle_info.param.yaml                  # Virtual values for differential drive
│   └── launch/robot_description.launch.py
└── kachaka_autoware_maps/                           # Sample map location (assets stay external)
    └── README.md                                     # Procedure for users to author maps
```

Dependencies of the new packages:

- `kachaka_autoware_vehicle_interface` → `autoware_control_msgs`, `autoware_vehicle_msgs`, `autoware_adapi_v1_msgs`, `geometry_msgs`, `nav_msgs`, `std_srvs`
- `kachaka_autoware_description` → `kachaka_description` (after the update), `xacro`, `robot_state_publisher`
- `kachaka_autoware_bridge` → the three above + `autoware_default_adapi`, `autoware_adapi_adaptors`, `autoware_core_*`

Updates to `kachaka_description`:

- Add `urdf/_shelf_3tier.urdf.xacro`, providing `xacro:macro name="shelf_3tier" params="parent shelf_name"`.
- Add shelf materials (`shelf_board` / `shelf_post`) to `urdf/_materials.urdf.xacro`.
- Do not touch existing `_kachaka.urdf.xacro` / `_values.urdf.xacro` / `kachaka.urdf.xacro` (avoid breaking changes so existing users' URDF output stays the same).

## 6. Localization Design

### 6.1 Inputs

- OS-1 128: published by the `ouster-ros` driver as `/sensing/lidar/top/pointcloud_raw_ex` (`sensor_msgs/PointCloud2`).
- Kachaka wheel odometry: existing `wheel_odometry_component` topic `/kachaka/wheel_odometry/wheel_odometry`.
  - The Vehicle Interface republishes it as `/vehicle/status/velocity_status` (`autoware_vehicle_msgs/VelocityReport`).
  - `vehicle_velocity_converter` converts it to `/sensing/vehicle_velocity_converter/twist_with_covariance`.
  - **To verify:** the design assumes Kachaka's internal `wheel_odometry` is generated from wheel encoders + IMU, but if the internal implementation also depends on the 2D LiDAR, the LiDAR's unavailability could affect it. Verify on the real robot during M2; if unusable, implement a fallback path that builds the twist locally from the IMU (`/kachaka/imu/imu`) and the angular-velocity component of Kachaka's `wheel_odometry`.

### 6.2 Configuration

Use `autoware_core_localization.launch.xml` largely as-is, with only the following parameter tweaks:

- `pose_initializer.param.yaml`:
  - `gnss_enabled: false`
  - `yabloc_enabled: false`
  - `ndt_enabled: true`
  - `pose_error_check_enabled: false`
  - `stop_check_enabled: true`
- `ndt_scan_matcher.param.yaml`:
  - Tune `initial_pose_estimation.particles_num` on the real robot.
  - `align_using_monte_carlo: true` (allow full-range search).
- Do not start any GNSS topic node (with `gnss_enabled: false`, `pose_initializer` does not block on GNSS, so an empty publisher on `/sensing/gnss/pose_with_covariance` is unnecessary). Override the launch arg `gnss_input_topic` of `autoware_core_localization.launch.xml` with an empty string.

### 6.3 Outputs

- `/localization/kinematic_state` (`nav_msgs/Odometry`) — the single source of truth used downstream.
- `/localization/acceleration` (`geometry_msgs/AccelWithCovarianceStamped`).
- TF: `map → odom`.

## 7. Planning Design

Use `autoware_core_planning.launch.xml` largely as-is, with the following adjustments:

- `vehicle_param_file`: the new `kachaka_autoware_description/config/vehicle_info.param.yaml`.
- `motion_velocity_planner_launch_modules`: `[]` for the MVP (ObstacleStop disabled). When enabling it in M6, feed the **OS-1 128 point cloud** into `/perception/obstacle_segmentation/pointcloud` (Kachaka's 2D LiDAR is unavailable). Ground removal can be handled with `autoware_crop_box_filter` or similar.
- Input perception topics (`/perception/object_recognition/objects`, `/perception/obstacle_segmentation/pointcloud`, `/perception/traffic_light_recognition/traffic_signals`, `/perception/occupancy_grid_map/map`, etc.) are to be **verified on the real robot** during M4 startup. If `behavior_velocity_planner` / `motion_velocity_planner` cannot start due to a missing topic, add a helper node `perception_stub` inside `kachaka_autoware_bridge` that publishes an empty message at 1 Hz (decision made in M4).

### 7.1 Goal Reception Flow

1. RViz `RouteTool` (autoware_rviz_plugins) calls `/api/routing/set_route_points`.
2. `autoware_default_adapi/routing` translates that into a call to `/planning/mission_planning/set_waypoint_route`.
3. `mission_planner` computes the route on lanelet2 and publishes `/planning/route`.
4. `path_generator` → `behavior_velocity_planner` → `motion_velocity_planner` → `velocity_smoother` → `/planning/trajectory`.

## 8. Control Design

Use `autoware_simple_pure_pursuit` **as-is**; do not fork it.

- Inputs: `/localization/kinematic_state`, `/planning/trajectory`.
- Output: `/control/command/control_cmd` (`autoware_control_msgs/Control`).
- The `wheel_base` in `vehicle_info` is a virtual value for the differential drive (described below).

## 9. Vehicle Interface Design (core component)

### 9.1 Node Overview

`kachaka_vehicle_interface_node` is implemented as a single node (`rclcpp::Node`) with the following responsibilities.

#### A. Control → Twist conversion

- Subscribes to: `/control/command/control_cmd`.
- Bicycle-model → differential-drive conversion:
  - `v = control.longitudinal.velocity`
  - `omega = v * tan(control.lateral.steering_tire_angle) / wheel_base`
- Output: `geometry_msgs/Twist`.
- Limits: `|v| ≤ 0.3 m/s`, `|omega| ≤ 1.57 rad/s` (consistent with the existing ManualControl limits).

#### B. cmd_vel publish gate

- Subscribes to `/system/operation_mode/state`; only when `mode == AUTONOMOUS` does it publish the Twist on `/kachaka/manual_control/cmd_vel`.
- In any other mode, publishing is suppressed entirely.

#### C. /vehicle/status/velocity_status publishing

- Subscribes to: Kachaka's `wheel_odometry` (existing) or `/odometry/odometry`.
- Conversion: `nav_msgs/Odometry` → `autoware_vehicle_msgs/VelocityReport`:
  - `longitudinal_velocity = twist.linear.x`
  - `lateral_velocity = 0.0` (differential drive)
  - `heading_rate = twist.angular.z`
- Rate: 50 Hz.

#### D. Manual Control auto-enable

- On node startup, call Kachaka's `/kachaka/manual_control/set_enabled` service with `true`.
- On node shutdown, call it with `false` (destructor / on_shutdown).

#### E. Operation Mode state machine

`autoware_default_adapi` in the `autoware_core` family does not include an `operation_mode` node, so a lightweight state machine is implemented inside the Vehicle Interface:

- States: `STOP` / `AUTONOMOUS` (only these two for the MVP).
- Services provided: `/system/operation_mode/change_to_autonomous`, `/system/operation_mode/change_to_stop`.
- Topic published: `/system/operation_mode/state` (`autoware_adapi_v1_msgs/OperationModeState`) at 10 Hz.
- Initial state at startup is `STOP`; transitions to `AUTONOMOUS` via the RViz EngageButton.
- To leave room for evolving toward scope C, the responsibilities are split across the submodules listed in 9.1 so that this node can later be replaced by `autoware_command_mode_decider` (Universe) by swapping out only the Operation Mode submodule.

### 9.2 Configuration

`config/vehicle_interface.param.yaml`:

```yaml
/**:
  ros__parameters:
    max_linear_velocity: 0.3
    max_angular_velocity: 1.57
    cmd_vel_timeout: 0.5         # [sec] stop if exceeded
    publish_period_velocity_status: 0.02  # 50 Hz
    publish_period_operation_mode: 0.1     # 10 Hz
    auto_enable_manual_control: true
```

### 9.3 vehicle_info.param.yaml (virtual values for differential drive)

```yaml
/**:
  ros__parameters:
    wheel_radius: 0.045
    wheel_width: 0.025
    wheel_base: 0.30          # virtual value, acts as the turn coefficient for pure_pursuit
    wheel_tread: 0.20         # actual value from the Kachaka URDF
    front_overhang: 0.237      # body collision +X: 0.0435 + 0.387/2
    rear_overhang: 0.150       # body collision -X: -(0.0435 - 0.387/2)
    left_overhang: 0.120       # body collision +Y: 0 + 0.240/2
    right_overhang: 0.120      # body collision -Y: -(0 - 0.240/2)
    vehicle_height: 1.20      # including the shelf
    max_steer_angle: 1.5708
```

`wheel_base` is a tuning parameter on the real robot. Start at 0.30; increase if turns are too sharp, decrease if they are too sluggish.

## 10. AD-API Integration

### 10.1 Launch

Use `autoware_core_api.launch.xml` as-is:

- `autoware_default_adapi` (`interface` / `localization` / `routing`)
- `autoware_adapi_adaptors` (`initial_pose_adaptor` / `routing_adaptor`)

### 10.2 RViz Panels

Clone and build `autoware_rviz_plugins` separately:

```bash
cd ~/ros/jazzy/src
git clone https://github.com/autowarefoundation/autoware_rviz_plugins.git
cd ~/ros/jazzy
colcon build --packages-select autoware_rviz_plugins
```

Panels used:

| Panel | Role |
|---|---|
| `InitialPoseButtonPanel` | Confirm the initial pose (kicks off NDT monte carlo) |
| `RouteTool` (or the standard `2D Goal Pose`) | Specify the goal → `/api/routing/set_route_points` |
| `EngageButton` | Transition to AUTONOMOUS |
| `AutowareStatePanel` | Display the current operation_mode |

### 10.3 Operating Sequence (MVP)

1. Launch `kachaka_autoware.launch.xml` on Thor.
2. Start RViz2 on the development PC and load the `autoware.rviz` configuration.
3. Press "Initialize" on `InitialPoseButtonPanel` → NDT monte carlo confirms the initial pose.
4. Specify the goal via `RouteTool` → confirm the trajectory is generated.
5. Press `EngageButton` to transition to AUTONOMOUS → the Vehicle Interface starts emitting cmd_vel and Kachaka begins moving.
6. On reaching the goal, the trajectory shortens, pure_pursuit issues a stop command, and the system automatically returns to the STOP mode.

## 11. Data Flow (key diagram)

```
OS-1 → /sensing/lidar/top/pointcloud_raw_ex
        ↓ (downsample) NDT scan_matcher
        ↓
        EKF ← Kachaka wheel_odometry → vehicle_velocity_converter
        ↓
   /localization/kinematic_state
        ↓
        Planning (mission_planner ← AD-API set_route_points)
        ↓
   /planning/trajectory
        ↓
   simple_pure_pursuit
        ↓
   /control/command/control_cmd  (autoware_control_msgs/Control)
        ↓
   kachaka_vehicle_interface
        ↓ (only when operation_mode == AUTONOMOUS)
   /kachaka/manual_control/cmd_vel  (geometry_msgs/Twist)
        ↓ gRPC SetRobotVelocity
   Kachaka body
```

## 12. Error Handling / Fail-safe

| Anomaly | Detection | Action |
|---|---|---|
| Control msg receive timeout | Vehicle Interface exceeds `cmd_vel_timeout` | Publish a zero Twist for 1 second to stop |
| Operation Mode != AUTONOMOUS | Vehicle Interface | Suppress publishing (no cmd_vel emitted) |
| Kachaka SetRobotVelocity rejected | gRPC `kErrorCodeApiGrpcSetRobotVelocityNotInTeleopMode` | Existing `ManualControlComponent` retry |
| NDT score degradation | (post-MVP) monitor `exe_time_ms` / score | Implemented in M6; manual monitoring for now |
| Single-node crash | Standard rclcpp | Restart via systemd / launch lifecycle (operations design) |

## 13. Test Strategy

### 13.1 Unit Tests (gtest)

- `kachaka_autoware_vehicle_interface`:
  - Boundary cases of the Control → Twist conversion (v=0, δ=0, δ=max).
  - Operation Mode state transitions (STOP → AUTONOMOUS → STOP).
  - Zero-Twist emission on cmd_vel timeout.
  - velocity_status conversion.

### 13.2 Integration Tests

- Record a ros2 bag (OS-1 + Kachaka odometry + control_cmd) and run regression tests on NDT/EKF output via offline replay.
- AD-API scenario tests (modeled on `autoware_default_adapi/test`).

### 13.3 System Tests (real robot)

- In the home environment, try 5 different 2D Goal Poses, 3 attempts each.
- Success criterion: reach within ± 0.3 m / ± 0.2 rad of the goal pose, with no human intervention.
- On failure: keep a full rosbag.

## 14. Phased Build-up (Milestones)

| ID | Content | Done when |
|---|---|---|
| **M0** | Prerequisite work | OS-1-only `pointcloud_map.pcd` built / `lanelet2_map.osm` built (same origin) / OS-1 physically mounted and calibrated |
| **M1** | Sensor integration | OS-1 publishes on ROS 2, the TF tree is complete, RViz shows the point cloud relative to `base_footprint` |
| **M2** | Localization | NDT + EKF emit `/localization/kinematic_state` (verified with Kachaka stationary). Validate `wheel_odometry` on the real robot; if NG, implement the IMU fallback |
| **M3** | Vehicle Interface foundation | Control→Twist conversion, velocity_status, operation_mode state machine, Manual Control auto-enable |
| **M4** | Planning | mission_planner→trajectory generation (manually triggered) |
| **M5** | Closed loop | A single-point goal from AD-API + RViz makes Kachaka move (**MVP achieved**) |
| **M6** | Polish | OS-1-based obstacle stop / multi-waypoint / dock integration (post-MVP) |

## 15. Room for Future Extension (B → C)

The submodule split inside the Vehicle Interface (Control conversion / velocity_status / operation_mode) means that when migrating to a future Universe component such as `autoware_command_mode_decider`, **only the Operation Mode state-machine submodule** needs to be swapped out. The Control conversion and velocity_status remain because they are Kachaka-specific.

## 16. Risks and Open Issues

| Item | Risk | Mitigation |
|---|---|---|
| Tuning the virtual `wheel_base` | Turning behavior may not match intuition | Tune on the real robot in M5; document the value in param.yaml |
| Effort to build the pointcloud_map | M0 takes time; OS-1 must do it alone since Kachaka's 2D LiDAR is unavailable | Research best practices from lio_sam / fast_lio / glim users; map by pushing or teleoping the robot |
| Robustness of indoor NDT | Divergence on featureless walls; Kachaka's SLAM cannot serve as a fallback | Evaluate on the real robot in M2 and tune voxel_size as needed. NDT-failure fail-safe is automatic AUTONOMOUS release (pose-error check) |
| Validity of Kachaka `wheel_odometry` | Internal sensor fusion may be off due to the missing 2D LiDAR | Verify on the real robot in M2; if NG, implement a fallback that builds the twist locally from IMU + angular velocity |
| Heat and power for OS-1 128 + Thor | Continuous operation in shelf-mounted form factor | Handled separately in thermal/power design (out of scope for this spec) |
| Conflict with Kachaka's built-in navigation | The robot may start moving on its own via gRPC | Suppressed via `set_manual_control_enabled(true)` |
| OS-1 as the only obstacle-stop sensor | Kachaka's proximity sensors cannot be used because of the 2D LiDAR situation | In M6, feed OS-1 128 point cloud through ground removal into ObstacleStop. Ground-removal parameters need indoor tuning |

## 17. Open Questions (to be decided before implementation, not fixed by this spec)

- Which process on Thor hosts the Vehicle Interface node (co-located in `grpc_ros2_bridge_container` vs. a separate container)?
- Semantics of the Operation Mode state machine's `change_to_*` services when **multiple are called simultaneously**.
- Memory operation on Thor when the pointcloud_map is large.

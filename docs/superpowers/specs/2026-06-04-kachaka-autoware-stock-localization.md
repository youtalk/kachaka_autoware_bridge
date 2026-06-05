# Kachaka × Autoware Core on the stock robot (no 3D LiDAR) — Pivot Design

- Created: 2026-06-04
- Status: Draft (review)
- Supersedes (localization/map axis only): `2026-05-02-kachaka-autoware-core-design.md`
- Related: [autoware_core](https://github.com/autowarefoundation/autoware_core),
  [kachaka-api](https://github.com/pf-robotics/kachaka-api)

## 1. Purpose

Make the **main functions of Autoware Core (localization → planning → control)**
run on a **stock Kachaka that has no Ouster OS-1 (3D LiDAR) mounted**, reusing
**Kachaka's own localization and map**. The end goal is unchanged from the
2026-05-02 design — a 2D Goal Pose in RViz drives Kachaka through Autoware's
planning and control — but the localization and map layers are re-sourced from
the robot's built-in 2D-LiDAR SLAM instead of NDT on a 3D point cloud.

## 2. What changes vs. the 2026-05-02 design

The 2026-05-02 design assumes an Ouster OS-1 on an "Autoware shelf" and states
explicitly that *"Autoware NDT is the sole source of localization"* and the
Kachaka 2D LiDAR / built-in localization are **not** used. This document inverts
exactly that axis and leaves the rest intact.

| Layer | 2026-05-02 design (shelf / 3D LiDAR) | This pivot (stock Kachaka) |
|---|---|---|
| Sensor | Ouster OS-1 128 via `ouster_ros` | none added (Kachaka body only) |
| Localization | NDT + EKF on point cloud | **Kachaka SLAM via TF (pass-through)** |
| Map (localization) | `pointcloud_map.pcd` | not used |
| Map (planning) | hand-authored `lanelet2_map.osm` | **auto-generated minimal lanelet2** over Kachaka's map, refined later |
| Planning | `autoware_core_planning` | unchanged |
| Control | `autoware_simple_pure_pursuit` | unchanged |
| Vehicle Interface | `kachaka_autoware_vehicle_interface` | unchanged |
| AD-API | `autoware_default_adapi` + adaptors | unchanged |

The existing Ouster/NDT launches are **kept, not deleted** — the 3D-LiDAR shelf is
an *additional* configuration, so the two bring-ups live side by side
(`kachaka_autoware.launch.xml` = shelf, new `kachaka_autoware_stock.launch.xml` =
stock).

## 3. Verified findings (live probe, 2026-06-04, read-only)

Running only the gRPC bridge against the real robot (192.168.1.101) confirmed
the stock robot already publishes everything Autoware needs for localization:

| Kachaka output | Topic | Type | Frames |
|---|---|---|---|
| SLAM localization | `/tf` (`dynamic_tf`) | `tf2_msgs/TFMessage` | **`map → odom`** present |
| Odometry | `/kachaka/odometry/odometry` | `nav_msgs/Odometry` | `odom → base_footprint` |
| Wheel odometry | `/kachaka/wheel_odometry/wheel_odometry` | `nav_msgs/Odometry` | `odom → base_footprint` |
| 2D map | `/kachaka/mapping/map` | `nav_msgs/OccupancyGrid` (latched) | `map`, 0.025 m |
| Body URDF TF | `/tf`, `/tf_static` | — | `base_footprint → base_link → …` |
| cmd_vel in | `/kachaka/manual_control/cmd_vel` | `geometry_msgs/Twist` | — |
| manual enable | `/kachaka/manual_control/set_enabled` | `std_srvs/SetBool` | — |

`tf2_echo map base_link` resolves directly. **Kachaka's TF tree already matches
Autoware's `map → odom → base_link` convention.** Therefore:

- **No NDT, no EKF.** Running `autoware_ekf_localizer` would *conflict* by
  double-publishing `map → odom`.
- The "localization bridge" is a thin node that republishes
  `/localization/kinematic_state` (Autoware's source-of-truth `nav_msgs/Odometry`
  in the `map` frame) from Kachaka odometry (twist) + a TF lookup of
  `map → base_link` (pose). See `kachaka-localization-via-tf` project memory.

Operational note: the bridge container takes ~30–40 s to fully start;
`dynamic_tf` (the localization TF) is the **last** component up.

## 4. Stock data flow

```
Kachaka SLAM ── /tf: map→odom ──┐
Kachaka odom ── /tf: odom→base ─┤
Kachaka URDF ── /tf_static ─────┤
                                ▼
            kachaka_autoware_localization  (TF lookup map→base_link + odom twist)
                                │
                   /localization/kinematic_state  (nav_msgs/Odometry, map→base_link)
                                ▼
        Planning (mission_planner ← AD-API set_route_points; lanelet2 = auto-min)
                                │
                       /planning/trajectory
                                ▼
                   autoware_simple_pure_pursuit
                                │
                   /control/command/control_cmd
                                ▼
              kachaka_autoware_vehicle_interface  (only when AUTONOMOUS)
                                │
                  /kachaka/manual_control/cmd_vel → gRPC SetRobotVelocity
                                ▼
                            Kachaka body
```

## 5. New / changed components

### 5.1 `kachaka_autoware_localization` (new package)

- **Node** `kachaka_autoware_localization_node`: subscribes
  `/kachaka/odometry/odometry`; on each message looks up `map → base_link` from
  TF and publishes `/localization/kinematic_state` (`nav_msgs/Odometry`,
  `header.frame_id = map`, `child_frame_id = base_link`, pose from TF, twist from
  Kachaka odometry). Pose/twist covariances are fixed parameters.
- **Pure-logic library** `KinematicStateConverter` (`ToKinematicState(...)`):
  TF `Transform` + body `Twist` + params → `nav_msgs/Odometry`. Fully unit-tested
  (mirrors `ControlToTwistConverter`).
- Twist frame note: Kachaka odometry child is `base_footprint`; `base_footprint →
  base_link` is a fixed planar-invariant offset, so planar `(vx, wz)` is reused
  unchanged with `child_frame_id = base_link`.

### 5.2 `kachaka_autoware_bridge/launch/kachaka_autoware_stock.launch.xml` (new)

Stock bring-up: gRPC bridge (which itself launches the Kachaka body URDF /
`robot_state_publisher`) + `kachaka_autoware_localization`. No Ouster, no NDT, no
point-cloud map. Later milestones add planning / control / AD-API includes.

### 5.3 Minimal lanelet2 generator (M3)

A small tool/script that emits a minimal valid `lanelet2_map.osm` +
`map_projector_info.yaml` sharing **Kachaka's map-frame origin** (so map-frame
poses from Kachaka stay consistent with planning). Bring-up uses an auto-generated
lane graph; a real map is authored later in Vector Map Builder.

## 6. Milestones (each = one `feat/` branch + PR)

| ID | Feature | Done when |
|---|---|---|
| **M1** | Localization bridge | `kachaka_autoware_stock.launch.xml` on the real robot publishes `/localization/kinematic_state` tracking the robot, and RViz shows it localized on `/kachaka/mapping/map`. Converter unit-tested. |
| **M2** | Vehicle interface + control bring-up | Built `kachaka_autoware_vehicle_interface` + `autoware_core_control` move the robot: `control_cmd → Twist → /kachaka/manual_control/cmd_vel` under operation-mode gating (verified with a canned/teleop trajectory). |
| **M3** | Map + planning | Auto-generated minimal lanelet2 + `autoware_core_planning` produce `/planning/trajectory` from a manually set route; `/localization/initialization_state = INITIALIZED` wired for AD-API. |
| **M4** | Closed loop (MVP) | A 2D Goal Pose from RViz/AD-API drives Kachaka to the target through Autoware planning + control on stock hardware. |

## 7. Risks / open items

- **AD-API initialization state.** Kachaka is already localized, so there is no
  NDT init step. AD-API/state expects `/localization/initialization_state =
  INITIALIZED`; M3 publishes it directly (no `pose_initializer`/NDT).
- **lanelet2 origin alignment.** The auto-generated lanelet2 must share Kachaka's
  map origin/orientation or planning poses will be offset. Verify in M3.
- **On-dock vs off-dock.** Kachaka's body LiDAR does not spin on the dock; `map →
  odom` may be static/degraded while docked. Localization verification (M1) and
  motion (M2+) are done off-dock.
- **Twist frame approximation.** `base_footprint` vs `base_link` planar twist
  assumed equal; valid because the offset is a fixed Z translation.
- **EKF stays off.** Do not launch `autoware_ekf_localizer` in the stock path; it
  would conflict with Kachaka's `map → odom`.
- **Map size.** Kachaka's home map is small (~6.9 × 4.35 m in the probe);
  trajectories and tuning must respect the tight indoor space.

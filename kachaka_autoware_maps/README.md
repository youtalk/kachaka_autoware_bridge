# kachaka_autoware_maps

Map preparation guide for running Kachaka with Autoware Core. Map artifacts
(`pointcloud_map.pcd` and `lanelet2_map.osm`) are not stored in this
repository; they live under `~/maps/<location_name>/` on the user machine.

## Prerequisites

- Ouster OS-1 128 (or any equivalent 3D LiDAR) is rigidly mounted on the
  Kachaka shelf top.
- The `ouster-ros` driver can publish point clouds and IMU samples.
- The pipeline uses the 3D LiDAR exclusively for both mapping and
  localization; Kachaka's built-in 2D LiDAR is not part of this stack.

## Directory layout

```
~/maps/<location_name>/
├── pointcloud_map.pcd
├── pointcloud_map/
│   └── metadata.yaml
├── lanelet2_map.osm
└── map_projector_info.yaml
```

Pass these paths to the `autoware_core_map.launch.xml` arguments
`pointcloud_map_path`, `pointcloud_map_metadata_path`, `lanelet2_map_path`,
and `map_projector_info_path`.

## Quick path: stationary snapshot (pipeline shake-out only)

For end-to-end launch verification before a real map exists, capture a
single OS-1 frame from the robot's current standstill pose. The
resulting map is **throwaway** — coverage is one viewpoint and Kachaka
cannot meaningfully navigate with it. Replace with a SLAM-built map
(sections 1–4 below) before driving.

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros/jazzy/install/setup.bash
ros2 launch kachaka_autoware_maps snapshot.launch.xml \
  sensor_hostname:=os-122609004411.local \
  output_dir:=$HOME/maps/kachaka_pipeline_test
```

The launch brings up `robot_description` (for the static
`base_footprint → os_lidar` chain) and the Ouster driver, captures one
`/ouster/points` message, transforms it to `base_footprint`, and writes:

- `pointcloud_map.pcd` (always overwritten)
- `pointcloud_map/metadata.yaml`, `lanelet2_map.osm`,
  `map_projector_info.yaml` (only if absent — user edits to the
  lanelet2 file are preserved on re-snapshot)

Then bring up the integrated launch with the throwaway map:

```bash
ros2 launch kachaka_autoware_bridge kachaka_autoware.launch.xml \
  sensor_hostname:=os-122609004411.local \
  map_path:=$HOME/maps/kachaka_pipeline_test
```

## 1. pointcloud_map.pcd

Run a 3D SLAM pipeline using the OS-1 alone. The recommended tool is
[`glim`](https://github.com/koide3/glim) on Jazzy. Drive Kachaka manually
through the entire mappable area and close the loop back to the start point.

## 2. lanelet2_map.osm

Use the [TIER IV Vector Map Builder](https://tools.tier4.jp/vector_map_builder_ll2/)
with the pointcloud_map loaded as background. Export with the same local
projection origin as the pointcloud_map. Lane width should be roughly
`0.6 m` (Kachaka body width 0.387 m + margin) and the speed limit
`0.3 m/s`.

## 3. map_projector_info.yaml

```yaml
projector_type: Local
vertical_datum: WGS84
```

`Local` is the GNSS-free indoor configuration. It assumes the
Vector Map Builder export used "Local Cartesian".

## 4. pointcloud_map/metadata.yaml (single-tile layout)

```yaml
x_resolution: 50.0
y_resolution: 50.0
pointcloud_map.pcd: [0, 0]
```

The map filename in the metadata must match the actual PCD file in the
layout above. Refer to the `autoware_map_loader` documentation for the
full schema if you later split the map into multiple tiles.

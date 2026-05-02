# kachaka_autoware_bridge

Autoware Core integration packages for the [Kachaka](https://kachaka.life/) indoor mobile robot. This repository ships:

- `kachaka_autoware_bridge` — meta-package with launch files that wire localization, planning, control, and the AD-API together for Kachaka.
- `kachaka_autoware_description` — URDF and meshes for the "Autoware shelf" (Jetson Thor + Ouster OS-1 128 mounted on top of Kachaka).
- `kachaka_autoware_maps` — pointcloud and lanelet2 maps used by the integration.
- `kachaka_autoware_vehicle_interface` — node and pure-logic libraries that adapt Autoware's vehicle-interface contract to Kachaka's gRPC bridge.

The [kachaka-api](https://github.com/pf-robotics/kachaka-api) repository is consumed as a git submodule pinned to a fixed release; this repo does not modify it.

## Quick start

```bash
mkdir -p ~/autoware_ws/src && cd ~/autoware_ws/src
git clone --recursive https://github.com/youtalk/kachaka_autoware_bridge.git
cd ~/autoware_ws
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
```

See `kachaka_autoware_bridge/launch/` for entry-point launch files and the design document at `docs/superpowers/specs/2026-05-02-kachaka-autoware-core-design.md` for the architecture.

## License

Apache-2.0. See [LICENSE](LICENSE).

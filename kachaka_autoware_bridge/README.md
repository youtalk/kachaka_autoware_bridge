# kachaka_autoware_bridge

Integration launches connecting the Kachaka gRPC ROS 2 bridge to Autoware Core.

## Stock closed loop (no 3D LiDAR) — drive a lap

Prerequisites: packages built (`colcon build`), `source install/setup.zsh`,
Kachaka reachable (verify the DHCP IP; default `192.168.1.101:26400`).

1. **Bring up bridge + localization and drive off the dock** (the body LiDAR
   only spins off-dock):

   ```bash
   ros2 launch kachaka_autoware_bridge kachaka_autoware_2d.launch.xml \
     server_uri:=192.168.1.101:26400 launch_control:=false
   ```

   Disable auto-homing, enable manual control, nudge forward to exit the dock.

2. **Generate the loop map** from Kachaka's occupancy map:

   ```bash
   ros2 launch kachaka_autoware_maps generate_loop_map.launch.xml \
     output_dir:=$HOME/maps/kachaka_loop
   ```

   Confirm the robot is on/inside the loop (see `loop_params.yaml`).

3. **Bring up the full stack** (localization + control + planning + AD-API +
   RViz):

   ```bash
   ros2 launch kachaka_autoware_bridge kachaka_autoware_2d.launch.xml \
     server_uri:=192.168.1.101:26400 \
     launch_control:=true launch_planning:=true launch_api:=true launch_rviz:=true \
     map_path:=$HOME/maps/kachaka_loop
   ```

4. **Set a route**, either way:
   - **RViz:** click **2D Goal Pose** and place it on the ring (drag to set
     heading). The routing adaptor converts it to an AD-API route. (Do **not**
     use **2D Pose Estimate** — the stock path is localized by Kachaka's SLAM and
     ignores `/initialpose`; it cannot be set from RViz.)
   - **Full lap (one command):**

     ```bash
     ros2 run kachaka_autoware_bridge set_loop_route
     ```

     If the lap comes up short (the goal landed in the same lanelet as the
     start), increase the offset, e.g. `--ros-args -p behind_angle:=0.6`.
   Confirm `ros2 topic echo /planning/trajectory --once` shows a trajectory.

5. **Engage autonomy** to drive:

   ```bash
   ros2 service call /system/operation_mode/change_to_autonomous \
     autoware_adapi_v1_msgs/srv/ChangeOperationMode {}
   ```

   The robot drives the loop. **Stop** any time:

   ```bash
   ros2 service call /system/operation_mode/change_to_stop \
     autoware_adapi_v1_msgs/srv/ChangeOperationMode {}
   ```

Safety: the vehicle interface forwards velocity **only while AUTONOMOUS** and
has a zero-Twist watchdog; Kachaka also has a built-in front-obstacle stop.

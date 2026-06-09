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

## Endurance run (M5)

Long-term, unattended multi-lap test harness on the stock closed loop. Drives the
loop continuously while a seeded scenario scheduler varies speed, inserts
stop-and-go dwells, and periodically does a real goal arrival; recovers from
transient faults; halts-and-captures on hard faults; and stops gracefully on a
duration/operator (later: battery) signal.

Prerequisites: a generated loop map (see "Stock closed loop (no 3D LiDAR) — drive a
lap"), robot off-dock, packages built and `source install/setup.zsh`.

1. **Bring up the full stack + orchestrator:**

   ```bash
   ros2 launch kachaka_autoware_bridge endurance.launch.xml \
     server_uri:=192.168.1.101:26400 map_path:=$HOME/maps/kachaka_loop \
     seed:=12345 max_duration_sec:=1800
   ```

   The orchestrator drives onto the ring, then runs laps. With
   `max_duration_sec:=1800` it stops gracefully after 30 min; `0` runs until a
   stop is requested.

2. **Stop gracefully at any time:**

   ```bash
   ros2 service call /run_endurance/stop_graceful std_srvs/srv/Trigger {}
   ```

3. **Watch progress:**

   ```bash
   ros2 topic echo /run_endurance/event              # state transitions, steps, faults
   ros2 topic echo /diagnostics --once               # endurance_orchestrator status
   ```

   A session record JSON is written to `~/endurance_runs/` on exit (laps, stop
   reason, scenario histogram, seed, any fault info).

Safety: the vehicle interface forwards velocity **only while AUTONOMOUS** and
has a zero-Twist watchdog; Kachaka also has a built-in front-obstacle stop, so an
orchestrator crash cannot drive the robot.

The harness is shape-agnostic: point `map_path` at a rounded-rectangle map (see
`kachaka_autoware_maps` README) to exercise planner stops and left/right turns.
At each corner the robot stops at the stop line (`behavior_velocity
StopLineModulePlugin`), then turns; the circle map works unchanged.  The session
record JSON written to `~/endurance_runs/` includes the planner `stop_count`.

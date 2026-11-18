# Full rover, terrain perception, and Nav2 simulation test

This guide runs the driving rover on the supplied terrain STL with its own
LiDAR sensors, Gazebo ground-truth odometry, a rolling terrain-geometry window,
and terrain/obstacle/inflation layers in **both** Nav2 costmaps. A navigation goal
commands the steering and wheel controllers and moves the simulated rover.

Use `terrain_rover.launch.py` for this test. The earlier
`terrain_bench.launch.py` is a stationary perception bench;
`terrain_sim.launch.py` uses a synthetic cost patch. Neither is the full moving
test. Start this combined launch by itself in its ROS domain/Gazebo partition.

## 1. Dependencies and build

ROS 2 Jazzy, Gazebo Harmonic, and a working Ogre2 rendering context are required.
Headless mode still renders the GPU LiDAR. From the workspace:

```bash
cd ~/DEIMOS-ws/rover_ws
source /opt/ros/jazzy/setup.bash
sudo apt-get install ros-jazzy-navigation2 ros-jazzy-pointcloud-to-laserscan
rosdep install --from-paths \
  src/rover_description src/rover_swerve_controller src/rover_navigation \
  src/lidar_terrain_description src/terrain_geometry_physical/terrain_geometry \
  --ignore-src -r -y --rosdistro jazzy
colcon build
source install/setup.bash
```

If Nav2 or Gazebo exits with `symbol lookup error` mentioning
`diagnostic_updater::Updater`, update the matching control packages together.
This machine initially mixed January ros2_control binaries with November Nav2
binaries; rebuilding this workspace alone could not resolve that ABI mismatch:

```bash
sudo apt-get install \
  ros-jazzy-diagnostic-updater ros-jazzy-controller-manager \
  ros-jazzy-controller-manager-msgs ros-jazzy-controller-interface \
  ros-jazzy-hardware-interface ros-jazzy-realtime-tools \
  ros-jazzy-transmission-interface ros-jazzy-gz-ros2-control \
  ros-jazzy-forward-command-controller ros-jazzy-joint-state-broadcaster
```

Verify dependency resolution:

```bash
rosdep check --from-paths \
  src/rover_navigation src/rover_description src/rover_swerve_controller \
  src/lidar_terrain_description src/terrain_geometry_physical/terrain_geometry \
  --ignore-src --rosdistro jazzy
```

## 2. Terminal environment

Run these commands in **every** terminal used for the test:

```bash
cd ~/DEIMOS-ws/rover_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=185
export GZ_PARTITION=rover_full_scope
```

The separate domain/partition prevents another simulator's `/clock`, `/scan`,
TF, or cost-grid publisher from joining this test. Do not also launch rover
bringup, the stationary scanner, the synthetic terrain publisher, or a second
Nav2 instance in this domain.

## 3. Launch the complete simulation

Terminal 1:

```bash
ros2 launch rover_navigation terrain_rover.launch.py
```

This starts one terrain world, the actual rover model, ros2_control, steering
kinematics, two rover-mounted sensors, bridges, terrain processing and its cost
adapter, Nav2, a terrain-validity command gate, and RViz. The terrain tile supplies
both visual and collision geometry; no hidden flat floor is added.

For unattended testing:

```bash
ros2 launch rover_navigation terrain_rover.launch.py \
  gui:=false rviz:=false headless_rendering:=true
```

The rover starts at XY=(0,0), Z=0.5 m and settles onto the terrain. Wait for
all three joint controllers to become active, Nav2's `Managed nodes are active`
message, and `Publishing costs from fresh terrain observations`. Initial TF
waits and `no map received` warnings can occur while the robot and sensors start.

## 4. Inspect readiness before driving

Terminal 2:

```bash
ros2 run rover_navigation test_rover_navigation.py
```

Without movement flags, this checks the live topics, finite 3D returns, both
costmaps, dynamic TF, active wheel/steer controllers, and Nav2 lifecycle states.
It prints a JSON report ending with `"result": "PASS"` when ready.

Useful individual checks:

```bash
ros2 control list_controllers
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /bt_navigator
ros2 topic echo /terrain/cost_grid_valid --once
ros2 topic echo /rover/lidar/points --field header --once --qos-reliability best_effort
ros2 topic echo /scan --field header --once --qos-reliability best_effort
ros2 topic echo /odom --once
ros2 topic info /terrain_cost_grid --verbose
ros2 run tf2_ros tf2_echo odom terrain_lidar_link
```

Stop `tf2_echo` with Ctrl+C. To measure rates, run `ros2 topic hz` on
`/rover/lidar/points`, `/scan`, `/terrain/grid`, and `/odom`, stopping each with
Ctrl+C before running the next command. The 3D scanner is configured for 5 Hz,
the horizontal scanner for 10 Hz, and terrain processing for 1 Hz; measured wall
rates depend on real-time factor and CPU/GPU load.

## 5. Test actual driving and goal execution

The automated test deliberately moves the simulated rover. Start from a fresh
launch for repeatable results:

```bash
ros2 run rover_navigation test_rover_navigation.py \
  --drive --navigate --goal-distance 1.5 --pause-test \
  --report /tmp/terrain_rover_validation.json
```

The test checks:

1. Three active joint controllers and six active Nav2 lifecycle nodes.
2. Finite rover-mounted LiDAR data, `/scan`, dynamic odometry/TF, graded terrain
   costs, and unknown areas in the terrain input/global costmap.
3. Six seconds of forward command through the smoother, collision monitor,
   terrain gate, and wheel controllers; odometry must show displacement.
4. A relative 1.5 m `NavigateToPose` goal in the current heading. The action must
   succeed, odometry must change, and final goal error must be below 0.35 m.
5. A simulator pause: terrain expires to unknown, validity becomes false, and
   nonzero probe commands into the gate are blocked at both `/cmd_vel` and wheel
   commands. The world resumes and fresh terrain must recover.

The pause test resumes the world in cleanup. Failure produces a nonzero exit
code and a partial report rather than a PASS. A blocked/unknown goal is a valid
navigation refusal, but fails this particular short-goal acceptance test.

For an interactive navigation test, use RViz's **Nav2 Goal** tool. Start with a
nearby goal on observed, low-cost terrain. The fixed frame is `odom`; ground-truth
pose already exists, so an AMCL initial-pose click is not required. Observe the
green global path, `/odom`, and the rover's wheels while the goal executes.

The default view looks down at the rover with a **1 metre grid**. Choose a goal
only one or two grid squares away for the first test. Click and drag the arrow
to set the final heading. The full STL and raw returns can extend far beyond
the terrain processor's current window; their visibility does not establish a
valid navigation destination. They start disabled in the navigation view and
can be enabled from Displays for perception inspection.

Use **Nav2 Goal** for a single destination. `NavigateThroughPoses` requires an
actual non-empty waypoint list; starting it without points aborts with
`No viapoints given`. **Reset** in the Navigation 2 panel takes the lifecycle
nodes out of service. If it was pressed, use **Startup** to reactivate Nav2
before sending another goal. Pressing Reset is not needed between goals.

To restore navigation after a reset from a terminal:

```bash
ros2 service call /lifecycle_manager_navigation/manage_nodes \
  nav2_msgs/srv/ManageLifecycleNodes '{command: 0}'
ros2 lifecycle get /bt_navigator
```

Wait for `active`. An unavailable action server in this setup can mean the
nodes were reset; RViz's generic question about an initial pose does not imply
AMCL needs configuring here.

Manual drive commands can be sent through the same monitored command chain.
Cancel an active navigation goal before using manual commands:

```bash
ros2 service call /navigate_to_pose/_action/cancel_goal action_msgs/srv/CancelGoal '{}'
ros2 topic pub --rate 10 /cmd_vel_nav geometry_msgs/msg/Twist \
  '{linear: {x: 0.1}, angular: {z: 0.0}}'
```

Stop publication with Ctrl+C, then publish zero:

```bash
ros2 topic pub --once /cmd_vel_nav geometry_msgs/msg/Twist '{}'
```

## 6. Inspect the perception and costmaps in RViz

The supplied `config/terrain_rover.rviz` starts with a close top-down camera
following `base_link`; the fixed frame remains `odom`. Scene mesh and raw
returns are optional inspection overlays. Its displays include:

| Display | Input | What to check |
| --- | --- | --- |
| Rover | `/robot_description`, `/joint_states`, TF | Model and wheel motion match odometry |
| STL terrain | `/terrain_scene/mesh` | Same tile used for Gazebo collision |
| Simulated returns | `/rover/lidar/points` | Cloud stays aligned with terrain as the rover moves |
| Terrain costs | `/terrain_cost_grid` | Sensor-derived rolling grid, including unknown cells |
| Global costmap | `/global_costmap/costmap` | Graded terrain plus obstacles and inflation |
| Local costmap | `/local_costmap/costmap` | Nearby terrain/obstacles follow the moving rover |
| Obstacle scan | `/scan` | Horizontal sensor returns align with nearby obstacles |
| Global path | `/plan` | Planned route and replanning toward the goal |

Show one costmap overlay at a time. Terrain and costmap inputs use reliable,
transient-local subscriptions; sensor displays use best effort. The local
window can be fully observed and contain no unknown cells; the larger global
window should retain unknown space beyond the sensed region.

### STL area and rainbow terrain geometry

For the terrain inspection preset, launch with:

```bash
ros2 launch rover_navigation terrain_rover.launch.py rviz_view:=terrain
```

If the simulation is already running, load
`rover_navigation/config/terrain_inspection.rviz` with RViz's **File > Open
Config** instead of starting another simulation. The source file is
`src/rover_navigation/config/terrain_inspection.rviz` in this workspace.

This preset enables the area **STL terrain** mesh and the processed
**Terrain geometry - rainbow height (m)** display on `/terrain/grid_points`.
It colors each fitted ground cell by its Z height using the rainbow palette.
Spheres keep the cell centers visible against the mesh surface. The colors
auto-scale to the current grid's height range, so the same color can represent
a different height after the window moves. Unknown cells are not colored as
measured terrain.

For other measurements, disable the height display and enable either
**rainbow slope (deg)** (0–20 degrees) or **rainbow roughness (m)** (0–0.06 m).
**Ground returns - rainbow height** optionally shows the measured ground
points instead of the fitted cell centers. Detected obstacle returns are red.

The **Views** panel includes **Processed terrain close-up** and **Full STL
tile** saved views. The fixed frame remains `odom`, and the camera follows
the rover. The STL extends beyond the processed terrain window; navigation
goals still need to be on observed costmap cells. Use `rviz_view:=navigation`
for the close top-down navigation preset (the default).

The raw terrain grid must preserve unknown ground. The moving simulation makes
one explicit exception **inside Nav2**: `terrain_layer.footprint_clearing_enabled`
is true for the robot's currently occupied 0.7 m-radius footprint. This assumes
the already placed simulated rover has support there and avoids the body shadow
blocking its own start. It does not fill the raw terrain grid or clear unseen
ground ahead. The stationary/physical perception profile retains footprint
clearing disabled. This simulation assumption is not validated for physical use.

## 7. Obstacle, unknown-space, and terrain-cost checks

For an additional obstacle scenario, start from a fresh world. Save this box:

```bash
cat > /tmp/terrain_test_box.sdf <<'EOF'
<sdf version="1.9">
  <model name="terrain_test_box">
    <static>true</static><pose>2.5 0 0.4 0 0 0</pose>
    <link name="box">
      <collision name="collision"><geometry><box><size>0.6 0.6 0.8</size></box></geometry></collision>
      <visual name="visual"><geometry><box><size>0.6 0.6 0.8</size></box></geometry></visual>
    </link>
  </model>
</sdf>
EOF
gz service -s /world/stl_terrain/create \
  --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 5000 \
  --req 'sdf_filename: "/tmp/terrain_test_box.sdf"'
```

The command should return `data: true`. Check the box appears in scan/point-cloud
returns and the costmaps, with an inflated region. Select an observed goal beyond
it in RViz. Nav2 should detour if an observed free route exists, or refuse/stop
if the route is blocked; a command topic alone does not prove avoidance. Restart
the launch to remove the box and reset the rover. This extra scenario is a
manual acceptance test, separate from the measured short-goal test below.

For unknown space, inspect beyond the rolling terrain window and attempt a goal
there. The planner has `allow_unknown: false`: refusal is expected. Do not fill
unknown cells or disable collision checks just to make the test pass.

For terrain grades, compare `/terrain/grid` slope/roughness with
`/terrain_cost_grid`. Defaults are free below 3 degrees/0.01 m RMS roughness and
lethal at 20 degrees/0.06 m; these are demonstration thresholds, not measured
rover limits. The test mesh does not necessarily contain a lethal slope in every
window. Cost differences can influence routing without making a region lethal.

## 8. Frames, data flow, and stop behavior

```text
terrain_map --known Gazebo world identity--> odom
  odom --dynamic Gazebo 3D odometry--> base_link
    base_link --> lidar_link              (horizontal /scan)
    base_link --> terrain_lidar_link       (3D /rover/lidar/points)

3D cloud -> terrain_geometry -> /terrain/grid -> terrain_cost_adapter
  -> /terrain_cost_grid -> terrain + obstacle + inflation in BOTH costmaps
/scan -> obstacle layers and collision_monitor

Nav2 /cmd_vel_nav -> velocity_smoother /cmd_vel_smoothed
  -> collision_monitor /cmd_vel_terrain_input
  -> terrain_velocity_gate /cmd_vel
  -> swerve_kinematics_node -> steer/wheel controller commands -> Gazebo motion
```

The terrain window is 12 x 12 m at 0.3 m resolution, centered using the scan-time
base pose. The odometry frame is gravity aligned; roll/pitch/Z of the base are
retained so clouds are transformed correctly over uneven ground. The 3D sensor
is mounted 1 m above `base_link`; the existing 2D scanner remains at 0.25 m.
Returns within a 0.65 m horizontal radius of the body are discarded as self
returns; no free ground is invented by that filter.

Gazebo ground-truth odometry supplies pose without SLAM or drift. The terrain
window seed uses the CAD wheel geometry's 0.2921 m base-to-ground clearance,
with a 0.15 m seed tolerance. This is a simulation starting point and can fail
where wheel support, terrain relief, or body attitude differ substantially.

The adapter invalidates terrain after approximately two seconds and publishes
`/terrain/cost_grid_valid`. The wall-time command gate outputs zero for false or
absent validity (heartbeat age >1.5 s) or missing commands (>0.5 s). The swerve
node independently zeros wheel commands when `/cmd_vel` stops for 0.5 s. These
timeouts also work when simulation time pauses. They complement Nav2 collision
checking; they do not turn this simulation into a certified motion supervisor.

## 9. Configuration and troubleshooting

| File | Purpose |
| --- | --- |
| `launch/terrain_rover.launch.py` | Combined scene, rover, perception, Nav2, gate, viewers |
| `config/terrain_moving.yaml` | Input, frame, rolling grid, seed and self filter |
| `config/nav2_moving_terrain.yaml` | Both terrain-aware costmaps and current-footprint policy |
| `terrain_geometry/config/nav2_adapter.yaml` | Slope/roughness cost thresholds and expiry |
| `config/terrain_rover.rviz` | Full-stack viewer |

Copy configuration files before tuning, then restart with explicit paths:

```bash
ros2 launch rover_navigation terrain_rover.launch.py \
  terrain_config:=/absolute/path/terrain_moving.yaml \
  adapter_config:=/absolute/path/nav2_adapter.yaml \
  nav2_params:=/absolute/path/nav2_moving_terrain.yaml
```

These nodes read processing and cost policies at startup; `ros2 param set` alone
does not reconstruct their cached policies. Keep simulation time and frame
settings consistent in custom files.

- **No motion:** check controller states, goal status, `/cmd_vel_nav`,
  `/cmd_vel_smoothed`, `/cmd_vel_terrain_input`, `/cmd_vel`, and the terrain
  validity heartbeat in that order. Check `/odom` changes, not only wheel commands.
- **Goal rejected/collision ahead:** inspect local obstacles, unknown ground,
  body shadow, and costmap freshness. Short waits/replans can occur while a
  rolling grid refreshes. Persistent refusal needs investigation, not a forced
  free-space patch. Cancel a goal before manual driving.
- **Feedback aborted / outside bounds:** inspect the planner log's goal
  coordinates. The global rolling costmap is 20 x 20 m, and measured terrain
  spans only the current 12 x 12 m window. A goal at X=-20 m from an origin
  start is outside the map even if the mesh is visible there. Try a one- or
  two-square goal on known costmap cells. Distant goals need a larger, observed
  navigation map, not just a wider camera view.
- **No viapoints given:** a waypoint action was sent without waypoints. Use
  Nav2 Goal for a single destination or populate the waypoint list first.
- **Action server unavailable after Reset:** use Startup, wait for
  `/bt_navigator` to become active, then send the goal.
- **No TF or cloud drops:** verify a dynamic `odom -> base_link`, both sensor
  frames, and matching simulation timestamps. Do not add a static base transform.
- **Grid stops moving:** check `rolling_window`, `tracking_frame: base_link`,
  and whether odometry actually changes.
- **Stale terrain/no valid costs:** inspect cloud timestamps, TF, processing
  duration, and real-time factor. Reduce processing load if the grid cannot stay
  within its age limit. Keep unknown ground unknown while diagnosing.
- **Blank RViz:** use fixed frame `odom`, the supplied config, and correct topic
  QoS. No `/map` or AMCL publisher is required by this odometry-based setup.
- **No RViz window:** the full launch defaults to `rviz:=true`. Rebuild and
  restart after updates; a previous launch-scope bug allowed the nested Nav2
  launch's `rviz:=false` to suppress the outer viewer. Child arguments are now
  scoped, and regression tests verify exactly one viewer is enabled. The rover
  remains stationary until a navigation goal or drive command is sent.
- **Controllers already loaded / failed to configure:** check for another
  controller manager or a leftover headless simulator in the same ROS domain.
  Stop the old simulation and restart the combined launch; spawners must connect
  to the controller manager belonging to the displayed rover.
- **GPU/SDF warnings:** confirm finite returns and matching frame IDs. Warning
  text alone neither proves failure nor validates the sensor.
- **Reset:** Ctrl+C the combined launch, wait for it to stop, then relaunch.
  Cancel an active goal first when collecting a controlled stop measurement.

## 10. Verification and limits

Run source tests after changes:

```bash
colcon test --packages-select terrain_geometry rover_navigation
colcon test-result --test-result-base build/terrain_geometry/test_results --verbose
colcon test-result --test-result-base build/rover_navigation/test_results --verbose
```

The moving test was exercised on this machine on 17 November 2026, including
real simulated displacement, a successful NavigateToPose action, live terrain
costs, rolling grid movement, and expiry/command blocking/recovery. Results are
recorded in [docs/ROVER_MOVEMENT_VALIDATION.md](docs/ROVER_MOVEMENT_VALIDATION.md).
The checker writes a fresh report when run with `--report`; do not treat an old
PASS file as evidence for a changed configuration.

This verifies a short simulation route. Physical L1 sensing, hardware motor
control, SLAM/localization, sustained performance, long autonomous routes, and
terrain hazard certification are outside this result. STL units/provenance and
rover CAD geometry remain provisional. A supplied surface mesh and generic GPU
LiDAR do not reproduce the Unitree L1 scan/noise model or validate ditch/material
detection. The optional box scenario and visual RViz inspection require their
own acceptance checks.

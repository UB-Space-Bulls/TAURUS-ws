# Space Bulls — Nav2 Migration and Validation (A1)

For the complete **moving rover + STL terrain + mounted 3D LiDAR + terrain
geometry + both Nav2 costmaps + navigation** test, start with
[README_FULL_STACK.md](README_FULL_STACK.md). It includes the single combined
launch, a test that drives the rover and completes a Nav2 goal, RViz setup,
dependency checks, and stop/recovery tests.

For the area STL with rainbow terrain geometry, use
`ros2 launch rover_navigation terrain_rover.launch.py rviz_view:=terrain`.
An existing RViz window can load `config/terrain_inspection.rviz` without
restarting the simulation. The full guide explains the height, slope,
roughness, and saved camera-view options.

## Terrain perception integration (17 November 2026)

Imported `rover_navigation` from the `nav2` branch at commit
`730c412f3fab6b3c654cf606098fa659fa3241e7`, retaining the existing workspace
and its perception sources. The new stationary bench connects the supplied
terrain mesh and simulated 3D LiDAR to the real terrain processor, cost adapter,
scan converter, and Nav2. It does not start the synthetic `a1_terrain_grid` node.

Install dependencies and build:

```bash
cd ~/DEIMOS-ws/rover_ws
source /opt/ros/jazzy/setup.bash
sudo apt-get install ros-jazzy-navigation2 ros-jazzy-pointcloud-to-laserscan
# Required if a newly installed Nav2 reports a missing diagnostic_updater symbol:
sudo apt-get install --only-upgrade ros-jazzy-diagnostic-updater
rosdep check --from-paths src/rover_navigation \
  src/terrain_geometry_physical/terrain_geometry src/lidar_terrain_description \
  --ignore-src --rosdistro jazzy
colcon build
source install/setup.bash
```

Run the standalone stationary bench with Gazebo and RViz:

```bash
export ROS_DOMAIN_ID=184
export GZ_PARTITION=rover_terrain_bench
ros2 launch rover_navigation terrain_bench.launch.py
```

RViz starts in `terrain_map` with the terrain mesh, 3D returns, and terrain costs.
Toggle **Global costmap**, **Local costmap**, or **Obstacle scan** to inspect each
output; show one costmap overlay at a time. The costmap topics are
`/global_costmap/costmap` and `/local_costmap/costmap`.

For a server without viewers:

```bash
ros2 launch rover_navigation terrain_bench.launch.py \
  gui:=false rviz:=false headless_rendering:=true
```

In a second terminal with the same workspace and environment:

```bash
source ~/DEIMOS-ws/rover_ws/install/setup.bash
export ROS_DOMAIN_ID=184
export GZ_PARTITION=rover_terrain_bench
ros2 run rover_navigation check_terrain_integration.py \
  --pause-test --report /tmp/terrain_nav2_validation.json
```

`--pause-test` pauses this Gazebo world, verifies expiry to unknown, resumes it,
and checks recovery. Omit it for a read-only inspection plus a planner request.
The checker computes a path between observed cells; it does not send a motion
goal. Successful checks print a JSON report with `"result": "PASS"`.

The bench supplies a known fixed `terrain_map -> odom -> base_link` fixture and
zero-motion `/odom`. The simulation independently supplies
`terrain_map -> terrain_sim_lidar` at Z=1.2 m. These transforms describe only
the stationary bench. For a moving rover, use its actual odometry/localization
and sensor transforms; do not launch this fixture alongside rover bringup.

Data flow:

```text
/sim/terrain/cloud (PointCloud2, terrain_sim_lidar, best effort)
  -> terrain_node -> /terrain/grid (TerrainGrid, terrain_map)
  -> terrain_cost_adapter -> /terrain_cost_grid (OccupancyGrid, terrain_map)
  -> terrain_layer in BOTH Nav2 costmaps (odom)

/sim/terrain/cloud -> pointcloud_to_laserscan -> /scan
  -> obstacle_layer in BOTH costmaps and collision_monitor
```

`config/nav2_perception_params.yaml` is derived from the branch's terrain profile
using `terrain_geometry_impl.nav2_config.physical_profile(..., use_sim_time=True)`.
It preserves the rover's geometry and controllers, adds terrain to both rolling
costmaps, keeps graded costs and unknown cells, and disables planning through
unknown terrain. The horizontal scan slice is configured separately in
`config/terrain_scan.yaml`; ground below that slice is evaluated by terrain
perception. The bench processes one 12 x 12 m terrain snapshot at 1 Hz.

The original `terrain_sim.launch.py` below remains a **synthetic terrain demo**.
Use `terrain_bench.launch.py` for LiDAR-derived costs. Run only one publisher of
`/terrain_cost_grid` in a ROS domain.

See [the measured results and remaining limits](docs/TERRAIN_VALIDATION.md).

## Overview

This package configures Nav2 for the Space Bulls rover
simulation using ROS 2 Jazzy.

The current simulation uses odom as its navigation reference
frame. Global map localization is not yet implemented.

## Requirements

- ROS 2 Jazzy
- Nav2
- RViz2
- Gazebo Harmonic
- Docker development environment (recommended)
- WSL2/WSLg for graphical use on Windows

The repository's docker/dockerfile.dev provides the main
ROS 2 dependencies.

## 1. Start Development Environment

From the repository root:

```bash
docker build -f docker/dockerfile.dev -t rover-dev .
./docker/run-dev.sh
```

The Docker image only needs rebuilding when its dependencies
or Dockerfile change.

## 2. Build Workspace

Inside the Docker container:

```bash
cd /workspaces/rover_ws
colcon build --symlink-install
source install/setup.bash
```

## 3. Launch Rover Simulation

In the first container terminal:

```bash
ros2 launch rover_description gazebo.launch.py
```

This launches the Gazebo simulation, robot description,
controllers, odometry, and sensor interfaces.

## 4. Launch Nav2

Open a second WSL terminal:

```bash
docker exec -it rover-dev-container bash
```

Then launch navigation:

```bash
ros2 launch rover_navigation nav2_sim.launch.py
```

This starts the configured Nav2 nodes and RViz.

## 5. Verify Navigation Interfaces

In a container terminal, check:

```bash
ros2 topic list
ros2 topic echo --once /odom
ros2 topic echo --once /scan
ros2 run tf2_ros tf2_echo odom base_link
```

Stop tf2_echo using Ctrl+C.

Check Nav2 actions:

```bash
ros2 action list
```

## 6. Test Navigation in RViz

1. Confirm the RViz Fixed Frame is odom.
2. Verify the rover model and TF displays.
3. Inspect local and global costmaps.
4. Select the Nav2 Goal tool.
5. Send a reachable navigation goal.
6. Observe rover movement and goal completion.
7. Test additional goals and obstacle behavior.

A successful launch alone does not establish that
navigation or obstacle avoidance works correctly.

## Coordinate Frames

Current A1 configuration:

- RViz Fixed Frame: odom
- BT Navigator global frame: odom
- Global costmap frame: odom
- Local costmap frame: odom
- Robot base frame: base_link

The simulation must provide odom -> base_link TF.

A map -> odom transform is not required for this
odometry-based setup. Global localization can be
integrated later.

## Troubleshooting

ROS 2 command not found:
- Enter the Docker development container.

RViz frame errors:
- Verify odom -> base_link is being published.

Missing costmap or obstacle data:
- Verify /scan and /odom are publishing.

Navigation goals fail:
- Check Nav2 lifecycle states, TF, action servers,
  and velocity command connections.

Map display warnings:
- The default RViz configuration includes a /map
  display. No /map publisher is required for A1's
  current odometry-only setup.

## Known Limitations

- Navigation uses odometry rather than global localization.
- Odometry drift may accumulate over time.
- Gazebo must be launched separately from Nav2.
- Runtime behavior must be validated in simulation.

## Package Files

- launch/nav2_sim.launch.py
- config/nav2_params.yaml
- rviz/nav2.rviz
- package.xml

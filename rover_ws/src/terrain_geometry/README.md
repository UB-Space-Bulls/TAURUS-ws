# terrain_geometry — physical terrain only

Subscribes to an already-running PointCloud2 publisher. Calculates a stationary
grid of ground height, slope, roughness and unknown cells, plus separated ground,
obstacle and unclassified points. **It never launches a LiDAR driver or opens a
serial port.** This physical edition has no Gazebo dependencies, worlds or demo
launch. Your earlier simulation ZIP is retained separately.

The ROS package and message names remain `terrain_geometry` and
`terrain_geometry/msg/TerrainGrid`. Replace the older source folder rather than
keeping two packages with the same name. See [MIGRATION.md](docs/MIGRATION.md).

## Nav2 integration (v0.5.0)

The optional adapter now feeds the existing DEIMOS nav2 branch costmap interface.
Use `nav2_adapter:=true` on terrain.launch.py. It publishes `/terrain_cost_grid`
and a freshness heartbeat. See [NAV2_INTEGRATION.md](docs/NAV2_INTEGRATION.md)
for the reviewed branch/commit, config generation and required robot inputs.
The L1 driver remains independent.

## Build independently

```bash
source /opt/ros/jazzy/setup.bash
cd ~/DEIMOS-ws/rover_ws
rosdep install --from-paths src/terrain_geometry --ignore-src -r -y --rosdistro jazzy
colcon build --packages-select terrain_geometry
source install/setup.bash
```

Start `unitree_l1_driver` separately and verify raw points first. Any compatible
PointCloud2 publisher can be used; there is no dependency on that driver package.
Runtime input needs x/y/z float fields in metres, a nonempty source frame, stamps
compatible with the ROS clock, and a valid TF to a gravity-aligned target frame.
Nonfinite points are filtered before transforms. No IMU attitude estimator is
included; a tilted sensor frame must not be used as the level reference.

## Create measured configuration

Copy the terrain settings, or keep your already measured/tuned files:

```bash
cp -n src/terrain_geometry/config/terrain.yaml ~/l1_terrain.yaml
ros2 run terrain_geometry create_mount --output ~/l1_mount.yaml
```

The mount helper asks separately for height, roll, pitch and yaw, rejects blank
or nonfinite numbers, and refuses to overwrite an existing file unless passed
`--force`. You can reuse your existing valid mount file instead. Angles are
radians and height is metres. The target's XY origin is directly below the L1,
and the floor is z=0. Measure height from the L1 frame origin at its bottom
mounting surface. Correct x/y manually if using a different world origin.

A sensor exactly inverted about its X axis has roll=3.141592653589793,
pitch=0 and yaw=0. Use that only if it matches the physical installation. Point
the coverage toward the ground and keep the sensor stationary. A static TF does
not track rover movement. The template `config/l1_mount.example.yaml` deliberately
contains nulls for unmeasured height/orientation.

## Start terrain, after the driver

```bash
ros2 launch terrain_geometry terrain.launch.py \
  mount_file:=$HOME/l1_mount.yaml \
  config:=$HOME/l1_terrain.yaml
```

Defaults: `input_topic:=/unilidar/cloud`, `target_frame:=terrain_map`,
`lidar_frame:=unilidar_lidar`, `rviz:=true`. Hardware time is always selected
(`use_sim_time=false`). With a valid robot TF tree already running, omit
`mount_file` and set the actual gravity-aligned `target_frame` and source
`lidar_frame`. Never publish a second transform for an already-owned child frame.

The default grid covers x/y=[-6,6) m with 40x40 cells at 0.3 m resolution.
Processing is capped at 2 Hz and uses only the latest cloud, without accumulation.
A known ground seed is required: `seed_height` is the floor elevation in the
level frame, and `seed_x/seed_y/seed_radius` must overlap observed ground. Ground
absence leaves cells unknown; it does not make obstacles or missing returns safe.

## Observe and verify

```bash
timeout 10s ros2 run tf2_ros tf2_echo terrain_map unilidar_lidar
timeout 10s ros2 topic hz /terrain/grid
ros2 topic echo /terrain/grid --field status --once
```

RViz starts in `terrain_map`. Ground is green, obstacles red and unclassified
returns yellow. Unknown ground is gray. Enable height, slope or roughness grid
layers one at a time to avoid overlapping colors. A disabled raw-cloud display
is included for input comparison; select your topic there if you changed it.

Use a measured level floor, ramp and raised object to assess real outputs. The
configured classification slope bound and preview costs are not rover safety
limits. Unknown cells are expected with sparse scans or occlusion. Inspect the
mount, seed region and cloud density before adjusting thresholds. Reliable ditch
detection and material recognition are outside this method.

No input for 5 seconds produces a terminal warning. TF failures or stale/future
clouds also produce warnings. If you see a missing-serial-port error, it belongs
to the separate driver. Fix raw data before tuning terrain.

## Record and share

```bash
ros2 bag record -o l1_terrain_test \
  /tf /tf_static /unilidar/cloud /unilidar/imu /diagnostics \
  /terrain/grid /terrain/ground_points /terrain/obstacle_points \
  /terrain/unclassified_points /terrain/grid_points /terrain/unknown_cells \
  /terrain_cost_grid /terrain/cost_grid_valid
```

Save both YAML files and measured test geometry with the recording. Output stamps
preserve the cloud header; the included L1 driver uses host receipt time. Nav2
consumers must expire stale grids after input stops. The proposed contract is in
[NAV2_GRID_CONTRACT.md](docs/NAV2_GRID_CONTRACT.md); the optional Nav2 adapter is documented above.
See [VALIDATION.md](docs/VALIDATION.md) for test status and hardware limits.

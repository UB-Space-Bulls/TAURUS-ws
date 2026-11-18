# FAST-LIO integration (v0.6.0)

Reviewed against DEIMOS `localization/fast-lio` commit
`7295ce36432453ebddb38e30611daf3bf9fc75f1` (13 November 2026), and
`perception/terrain-geometry` commit `8f897ade3b6f0ca8a8af60493655243aeeeabdcc`.
The localization branch pins Ericsii/FAST_LIO_ROS2 at
`2fffc570a25d0df172720bac034fbdb6a13d2162` through `third_party.repos`.
No localization source, calibration, controller, or TF edge is changed here.

## Where to put this

Replace `rover_ws/src/terrain_geometry` with the `terrain_geometry` folder in
this ZIP. Keep your measured YAML files separately. Do not leave another copy
with a package.xml under `src`. The `unitree_l1_driver` package stays separate.
Use the team's localization branch/worktree where `rover_localization` and its
pinned FAST-LIO dependencies are already available. This ZIP does not contain
those dependencies and is not a full rover workspace.

```bash
source /opt/ros/jazzy/setup.bash
cd ~/DEIMOS-ws/rover_ws
rosdep install --from-paths src/terrain_geometry --ignore-src -r -y --rosdistro jazzy
colcon build --symlink-install --packages-up-to rover_localization terrain_geometry
source install/setup.bash
```

If FAST-LIO has not been imported/built yet, follow the localization branch's
workspace setup/third_party.repos instructions first. Do not overlay two
FAST-LIO versions or silently substitute the Livox driver for the L1 driver.

## Run with the recent simulation

Terminal 1 (one simulator only):

```bash
ros2 launch rover_localization fast_lio_test.launch.py rviz:=false
```

Let the rover settle stationary on a **verified level surface** before
FAST-LIO initialization. Confirm the initial body/IMU axes are level. Measure
the vertical distance from the `body`/IMU origin to local ground after settling;
this is not the LiDAR mount's 0.25 m extrinsic offset. The current simulation
checks that `body` coincides with `base_link`; physical hardware may differ.

Terminal 2, after sourcing the same workspace:

```bash
read -rp 'Measured body/IMU ground clearance in metres: ' BODY_CLEARANCE
ros2 launch terrain_geometry fastlio_terrain.launch.py \
  body_ground_clearance:="$BODY_CLEARANCE" \
  level_frame_confirmed:=true use_sim_time:=true
```

Do not set the confirmation flag without establishing the frame. FAST-LIO's
initial `camera_init` axes are not necessarily gravity-aligned: its initialization
estimates a gravity vector without guaranteeing a level world orientation.
For a tilted start, supply a calibrated, gravity-aligned fixed frame with a
correct timestamped transform to `camera_init`, and pass `target_frame:=that_frame`.
This package deliberately does not invent that correction.

Open RViz with Fixed Frame equal to the target frame. Add `/terrain/ground_points`,
`/terrain/obstacle_points`, `/terrain/grid_points`, `/terrain/unknown_cells` and
`/terrain/costmap_preview`.

## Data contract

| Input/output | Meaning |
| --- | --- |
| `/cloud_registered` | Current scan already in `camera_init`, stamped at FAST-LIO scan end |
| TF target frame ← `body` | Body position at exactly that same scan stamp |
| `/terrain/grid` | Latest processed scan; header retains source stamp; grid origin moves with body |
| `/terrain_cost_grid` | Optional existing Nav2 adapter output with the same frame and moving origin |

The 12×12 m default window moves in cell-sized XY increments. Its axes remain
world-aligned, so pitch/roll are not mistaken for ground slope. The seed follows
the actual body XY; its Z is body Z minus measured vertical ground clearance.
`min_z`/`max_z` are body-relative in rolling mode. The band must include ground;
for clearance over 1 m, lower `min_z` in a copied `fastlio_terrain.yaml` and pass
`config:=/absolute/path.yaml`. Ground-clearance variation on rough terrain can
invalidate the seed; tune only using observed data and preserve unknown cells.

Only scan-time TF is used, never TF's latest pose. An already-registered cloud
is not transformed by body pose a second time. Missing TF, invalid frame,
zero-stamped, stale or future data is dropped. Processing keeps only the newest
pending scan; no map or scan accumulation occurs. Do not use `/Laser_map` as input.
Restart localization and terrain after known localization resets or clock resets.

## Nav2 and hardware boundaries

`nav2_adapter:=true` enables the existing cost-grid adapter. It preserves the
moving origin and frame and expires stale observations. This does not connect
FAST-LIO's `camera_init → body` tree to Gazebo/Nav2's `odom → base_link` tree.
Before navigation, use an established localization/frame integration with one
valid TF authority per edge. An identity `camera_init → odom` transform is not
valid just because it makes RViz stop warning. Disable the synthetic
`a1_terrain_grid` producer before using real terrain costs. See NAV2_INTEGRATION.md
for unknown-space policy and heartbeat/crash limitations.

On real hardware use `use_sim_time:=false` and the actual tracking frame and
calibration. The current localization branch's real-hardware YAML contains
placeholders. Its Gazebo `lidar_type: 5` path has no true per-point timing;
physical Unitree L1 preprocessing/time fields, IMU units, synchronization and
extrinsics still require verification. This integration consumes FAST-LIO's
output; it does not make that upstream physical-L1 pipeline complete.

## Acceptance checks on the rover workspace

1. Verify increasing, nonzero `/cloud_registered` stamps and `camera_init` frame;
   inspect `/Odometry` (capital O), child `body`, and corresponding TF.
2. Stationary level terrain: check height/slope and compare expected clearance.
3. Slow translation: grid origin should follow body and ground remain in world
   coordinates. Turn in place: the grid must not rotate with heading.
4. Ramp: compare measured slope to known ramp; tilt startup requires explicit
   level-frame correction before this test is meaningful.
5. Stop cloud/TF publication: no fresh grids should be produced; with adapter
   enabled, `/terrain/cost_grid_valid` must go false and costs become unknown.
6. Reset Gazebo/FAST-LIO and restart terrain before continuing a new segment.

Offline tests verify geometry and adapter behavior. ROS/Gazebo runtime acceptance
and physical FAST-LIO/L1 operation were not run in the artifact-building environment.

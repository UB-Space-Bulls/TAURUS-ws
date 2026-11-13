# DEIMOS terrain costmap integration (v0.5.0)

## Repository review

Reviewed the public [DEIMOS-ws repository](https://github.com/UB-Space-Bulls/DEIMOS-ws)
on 13 November 2026. The relevant implementation is on branch **nav2**, at
[730c412f3fab6b3c654cf606098fa659fa3241e7](https://github.com/UB-Space-Bulls/DEIMOS-ws/commit/730c412f3fab6b3c654cf606098fa659fa3241e7),
committed 6 November: "Implement A1 obstacle navigation and terrain-aware routing".
The branch tip is not an ancestor of the reviewed main tip
`1f7c299f261233646cd334d366408f62efb618e1` (11 November, arm description update).
Use the nav2 branch's rover_navigation package; main does not contain this integration.

Relevant files under `rover_ws/src/rover_navigation`:

- `TERRAIN_GRID_INTERFACE.md`: `/terrain_cost_grid`, nav_msgs/OccupancyGrid,
  reliable/transient-local depth 1; values -1 unknown, 0..99 costs, 100 lethal.
- `scripts/terrain_cost_grid.py`: synthetic flat grid with a toggleable cost-80
  patch, not a physical terrain classifier. It initializes missing areas as free.
- `config/nav2_terrain_params.yaml`: a StaticLayer on the global rolling costmap,
  followed by scan obstacles and inflation. Local costmap only had scan/inflation.
  The planner allowed unknown cells and unknown-space tracking was not enabled.
- `launch/terrain_sim.launch.py`: starts the synthetic publisher and Nav2.
- `launch/nav2_sim.launch.py`: starts Nav2 with an overridable params_file and clock.

The interface document reports longer paths avoiding the synthetic patch. These
are the team's reported results, not a newly reproduced test. No current real-L1
terrain adapter was found in this reviewed navigation implementation.

## What this update implements

`terrain_cost_adapter` subscribes to `/terrain/grid` and publishes the existing
`/terrain_cost_grid` interface. It copies the input frame, origin, resolution,
dimensions, row-major order and stamp. It never renames terrain_map to odom or
invents a transform. StaticLayer supports transforming a rolling costmap into the
grid frame when a valid TF connection exists.

- Obstacle evidence maps to 100 even if underlying ground is unknown.
- Observed, supported ground maps to 0..99 according to the larger normalized
  slope/roughness excess. Slope inputs are radians; configured thresholds are degrees.
- At the lethal slope/roughness threshold the value becomes 100.
- Missing/invalid ground stays -1. Zero slope/height/roughness are valid values.
- Default thresholds: free below 3 degrees and 0.01 m RMS roughness; lethal at
  20 degrees or 0.06 m. These are demonstration settings, not rover climbing limits.
- Inputs expire after 2 seconds by BOTH source stamp and steady elapsed time.
  Repeated stamps cannot renew them. Invalid, stale, future and out-of-order
  messages are rejected. Clock rollback invalidates the previous observation.
- While the adapter remains alive, expired data is replaced by a full unknown
  grid. Republish rate is 2 Hz; source stamps are not refreshed to hide old data.
- `/terrain/cost_grid_valid` publishes a Bool heartbeat indicating freshness.

`configure_nav2` reads the team's actual installed configuration and writes a
separate profile. It preserves the rover dimensions, frames, existing sensor
settings and controller settings. It changes both costmaps to terrain, obstacle,
inflation order, tracks unknown cells, retains graded costs, disables terrain and
obstacle footprint clearing, and prevents Navfn from planning through unknown.
Obstacle combination method 2 prevents scan rays from converting unknown terrain
into free ground. Positive obstacles in still-unknown terrain remain blocked by
the unknown policy; this is intentionally conservative.

Unsupported/custom plugins or filters are rejected for manual review rather than
silently overwritten. The input YAML is never modified. No new Nav2 C++ layer or
third ROS package is required. The L1 driver remains separate and unchanged.

## Install and build

Replace your existing `src/terrain_geometry` source with the updated folder,
keeping your backup outside `src`. Rebuild only after the new files are in place.
The driver installation does not change. For rover_navigation, use the team's
nav2 branch or a worktree of it without discarding local changes.

```bash
cd ~/DEIMOS-ws/rover_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src/terrain_geometry src/rover_navigation \
  --ignore-src -r -y --rosdistro jazzy
colcon build --packages-select terrain_geometry rover_navigation
source install/setup.bash
```

If changing from an old symlink build causes a directory collision, back up only
this package's build/install folders as described in MIGRATION.md and rebuild.
Do not launch after a failed build.

## Start physical sensing and cost output

Terminal 1: run your physical L1 driver, using its detected serial port:

```bash
ros2 launch unitree_l1_driver l1.launch.py port:=/dev/ttyUSB0 rviz:=false
```

Terminal 2: measured stationary terrain plus the optional adapter:

```bash
ros2 launch terrain_geometry terrain.launch.py \
  mount_file:=$HOME/l1_mount.yaml config:=$HOME/l1_terrain.yaml \
  nav2_adapter:=true
```

If terrain is already running, start only the adapter instead (not both):

```bash
ros2 launch terrain_geometry nav2_adapter.launch.py
```

For a different TerrainGrid producer using simulation time, use the standalone
adapter launch with `use_sim_time:=true`. This physical terrain.launch.py remains
wall-time only. Match clocks across all nodes and use source acquisition/receipt
semantics honestly; the bundled L1 driver supplies host receipt stamps.

## Generate the Nav2 profile

```bash
ros2 run terrain_geometry configure_nav2 --output ~/nav2_terrain_physical.yaml
```

The default input is the installed
`rover_navigation/config/nav2_terrain_params.yaml`. An explicit source path is
also accepted:

```bash
ros2 run terrain_geometry configure_nav2 \
  --input ~/DEIMOS-ws/rover_ws/src/rover_navigation/config/nav2_terrain_params.yaml \
  --output ~/nav2_terrain_physical.yaml --force
```

Use `--use-sim-time` only for a simulated source; the generated profile otherwise
uses wall time. Review thresholds in `config/nav2_adapter.yaml` and save a custom
copy if tuning. Supply it with `adapter_config:=/path/to/nav2_adapter.yaml` on
terrain.launch.py, or `config:=...` on the standalone adapter launch.

## Inspect the output before navigation

```bash
ros2 topic info /terrain_cost_grid --verbose
ros2 topic echo /terrain_cost_grid --field header --once \
  --qos-reliability reliable --qos-durability transient_local
ros2 topic echo /terrain/cost_grid_valid --once
```

There must be exactly ONE publisher on `/terrain_cost_grid`. Stop the synthetic
`a1_terrain_grid` node. **Do not use terrain_sim.launch.py with the real adapter.**

RViz has a disabled "Nav2 terrain cost input" Map display in the supplied terrain
configuration. Enable it to view the actual OccupancyGrid. A bench test can use
terrain_map as the fixed frame without any navigation/control nodes running.
Stop the terrain processor while keeping the adapter running: within approximately
2 seconds plus one 0.5-second publish interval, the map should become unknown and
cost_grid_valid should become false. Resume new scans and check recovery.

## Connect the team's Nav2 stack

Only after the existing rover bringup supplies `/odom`, `/scan`, and valid
`odom -> base_link` and sensor transforms, run:

```bash
ros2 launch rover_navigation nav2_sim.launch.py \
  params_file:=$HOME/nav2_terrain_physical.yaml use_sim_time:=false
```

Despite its name, this launch accepts wall time and does not start Gazebo itself.
It starts navigation/controller nodes and can produce velocity commands when a
navigation goal or behavior is requested. This is not needed just to view costs.
The L1 driver publishes PointCloud2, not `/scan`; the team's existing scan
conversion/localization work is still required for this navigation configuration.

Nav2 uses odom; the physical terrain defaults to terrain_map. These must be
connected by a measured valid TF chain. Alternatively process directly in odom
using valid sensor-to-odom TF and matching ground seed coordinates. Do not rename
headers or create an identity transform merely to suppress TF errors. A fixed
bench mount cannot represent a moving rover; pose estimation and a suitable
moving terrain window remain separate integration work.

In Nav2 RViz inspect `/global_costmap/costmap` and `/local_costmap/costmap`. Verify
unknown areas stay unknown, measured steep/rough cells cost more than flat cells,
and existing obstacles/inflation remain active. Robot shadow can leave unknown
cells under the start pose and prevent planning; do not erase them to force a path.

## Operational limits and required checks

The standard StaticLayer does not enforce an input timestamp timeout or heartbeat.
If the adapter itself crashes, freezes or is disconnected from Nav2, its last map
can persist. The adapter's expiry covers upstream sensor/terrain loss while it is
alive, not failure of the adapter/network. Before autonomous use, the navigation
supervisor must stop/cancel motion when the validity heartbeat is false OR absent,
and handle TF/localization loss. A running adapter is not a motion safety system.

Full Jazzy launch, lifecycle, TF and RViz tests still need to run on the ROS
computer. Hardware motion/navigation is not certified by offline tests. Keep
unknown ground unknown; no reliable ditch or material detection is claimed.

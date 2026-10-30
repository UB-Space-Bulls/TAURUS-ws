# Unitree L1 terrain geometry — one ROS 2 package

Drop the **`terrain_geometry` folder directly into your workspace's `src`**.
This package contains the terrain processor, its custom grid message, Unitree
L1 simulation settings, test worlds, launch files and RViz configuration.
It does not require a TAURUS checkout or changes to `rover_description`.

## Install and run

Prerequisites: Ubuntu 24.04, ROS 2 Jazzy, `colcon` and `rosdep`. The commands
below install the declared ROS/Gazebo dependencies; Gazebo Harmonic is the
target simulator. No Unitree hardware driver is needed for the simulated demo.

Extract `terrain_geometry.zip` **into `src`**, so the layout is:

```text
your_workspace/src/terrain_geometry/package.xml
your_workspace/src/terrain_geometry/CMakeLists.txt
```

From **your workspace root** (the directory containing `src`):

```bash
source /opt/ros/jazzy/setup.bash
rosdep update
rosdep install --from-paths src/terrain_geometry --ignore-src -r -y --rosdistro jazzy
colcon build --symlink-install --packages-select terrain_geometry
source install/setup.bash
ros2 launch terrain_geometry demo.launch.py
```

Use `l1_variant:=RM` for the RM range setting; the default is PM. Add
`rviz:=false` to omit RViz. The launch starts Gazebo, the cloud/IMU/clock bridge,
the fixed sensor transforms, the terrain node and RViz. The first demonstration
uses a stationary sensor with known ground orientation, without SLAM.

**Replacing the earlier two-package bundle?** Read
[MIGRATION.md](docs/MIGRATION.md) first to avoid duplicate source packages and
stale build artifacts. The custom type is now `terrain_geometry/msg/TerrainGrid`.

## What appears

The world includes real surface geometry: a flat floor, a **15° ramp**, an
uneven mesh and a **0.7 m raised box**. RViz displays separated ground and
obstacle points, height/slope/roughness layers, and unknown ground cells.

| Output | Contents |
|---|---|
| `/terrain/grid` | Ground height, slope, roughness, status and observation counts |
| `/terrain/ground_points` | Accepted ground points |
| `/terrain/obstacle_points` | Raised obstacle evidence |
| `/terrain/unclassified_points` | Returns without a reliable classification |
| `/terrain/grid_points` | Known cell centers with slope/roughness visualization fields |
| `/terrain/unknown_cells` | Gray unknown-ground markers |
| `/terrain/costmap_preview` | Diagnostic OccupancyGrid; not a Nav2 costmap plugin |

Grid defaults: 0.3 m resolution, 50×40 cells, lower corner `(-6,-6,0)` m,
level `terrain_map` frame, one replacement snapshot per processed cloud.
Unknown height/slope/roughness are NaN; slope is stored in radians. The complete
[Nav2 grid contract](docs/NAV2_GRID_CONTRACT.md) includes timestamps and indexing.

## Check the live demo

In a second terminal, source ROS and this workspace, then run:

```bash
ros2 topic hz /unilidar/cloud
ros2 run tf2_ros tf2_echo terrain_map unilidar_lidar
ros2 interface show terrain_geometry/msg/TerrainGrid
ros2 run terrain_geometry check_demo --seconds 20
```

Expected mount: `(0,0,1.2)` m, roll=180°. The inverted native L1 hemisphere
faces the ground. The checker compares flat ground, the known ramp, roughness,
box obstacle evidence and retained unknowns. To capture a bag and screen
recording, follow [RECORDING.md](docs/RECORDING.md).

## Use an existing cloud or a physical L1

If a sensor already publishes `/unilidar/cloud`, launch only the processor:

```bash
ros2 launch terrain_geometry terrain.launch.py \
  input_topic:=/unilidar/cloud target_frame:=your_level_frame \
  use_sim_time:=false config:=/absolute/path/to/your_terrain.yaml
```

Copy `config/terrain.yaml` and adjust the known ground seed and grid bounds for
your installation. Supply a measured TF chain from the cloud frame to the
gravity-aligned target frame. For an existing simulation use `use_sim_time:=true`.
The launch does not invent mounting transforms for a rover or physical device.

The optional `l1_hardware.launch.py` starts the separately installed official
Unitree L1 driver as well. That vendor SDK/driver is **not embedded** in this
single-package simulation bundle. See [UNITREE_L1.md](docs/UNITREE_L1.md).

## Tests and validation limits

After building:

```bash
colcon test --packages-select terrain_geometry
colcon test-result --verbose
```

Or from this source package directory, without ROS:

```bash
PYTHONPATH=. python3 -m unittest discover -s test -v
OPENBLAS_NUM_THREADS=1 python3 scripts/validate_offline.py --output /tmp/terrain_offline.json
```

The 23 offline tests pass, including PM/RM envelopes, mounting, geometry,
occlusion and unknown handling. Live colcon/ROS/Gazebo/RViz and physical L1
tests have not been run in the authoring environment; a live recording is
still pending. The bundled PNG and JSON are explicitly offline results.

The simulated sensor is a **regular-raster L1 coverage/point-budget
approximation**, not the L1's exact nonrepeating scan pattern. It uses 240×90
attempted rays at 1 Hz and chosen synthetic range noise. The IMU is exposed
but does not estimate terrain orientation. See the L1 notes for all limits.

The processor uses observed ground seeds, neighborhood plane fits and height
residuals. Missing returns remain unknown. No reliable ditch detection,
material classification, moving-map accumulation or motion compensation is
claimed. Grid thresholds need tuning before use with real data.

## Package contents and handoff

`msg/TerrainGrid.msg` is generated by this package's CMake build. The Python
implementation lives in `terrain_geometry_impl`; executable wrappers and that
module are installed together under `lib/terrain_geometry`. This keeps the
implementation separate from ROS's generated `terrain_geometry.msg` namespace.
No second package or `setup.py` is required.

The included `terrain_rover.sdf` is a terrain-only world for later integration;
it does not spawn a rover or attach a sensor. Generate source meshes/worlds
with `python3 scripts/generate_world.py` if the test scene changes.

- [Unitree L1 model and hardware setup](docs/UNITREE_L1.md)
- [Nav2 grid contract](docs/NAV2_GRID_CONTRACT.md)
- [Method and primary references](docs/METHOD.md)
- [Validation results](docs/VALIDATION.md)
- [Recording checklist](docs/RECORDING.md)
- [Migration from the earlier bundle](docs/MIGRATION.md)

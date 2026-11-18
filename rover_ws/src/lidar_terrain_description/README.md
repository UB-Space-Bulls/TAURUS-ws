# lidar_terrain_description

Standalone ROS 2 Jazzy / Gazebo Harmonic terrain environment built from your
uploaded **lidar.stl**. Inspection shows a landscape surface, not a LiDAR housing.
This package is separate from unitree_l1_driver and terrain_geometry. It does not
open real hardware, start Nav2, or modify the rover URDF.

## Install

Extract this ZIP so the package is `~/DEIMOS-ws/rover_ws/src/lidar_terrain_description`.

```bash
cd ~/DEIMOS-ws/rover_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src/lidar_terrain_description --ignore-src -r -y --rosdistro jazzy
colcon build --symlink-install --packages-select lidar_terrain_description && source install/setup.bash
```

## View the terrain in RViz only

```bash
ros2 launch lidar_terrain_description display.launch.py
```

The default is a detailed central tile, approximately 60 x 60 source units.
For the full landscape:

```bash
ros2 launch lidar_terrain_description display.launch.py mesh:=overview
```

Zoom out in RViz (roughly 1600 m camera distance with scale=1) to see the overview.
The mesh appears on `/terrain_scene/mesh` in `terrain_map`. No scans are produced
by the display-only launch. It publishes an identity static TF from `world` to
`terrain_map` so RViz can resolve its fixed frame. Use one launch at a time.

If RViz shows only a grid, keep **Global Options > Fixed Frame** set to
`terrain_map` and check **STL terrain > Topic** is `/terrain_scene/mesh` with
**Durability Policy** set to **Transient Local**. After updating the package,
rebuild it, source `install/setup.bash`, and restart the launch so RViz loads the
updated configuration.

## Gazebo with simulated LiDAR returns

```bash
ros2 launch lidar_terrain_description gazebo.launch.py
```

This opens the detailed tile in Gazebo and RViz, uses the mesh for both visual
and collision geometry, and starts a stationary generic GPU LiDAR. The LiDAR
has 720 horizontal x 64 vertical samples, 360-degree horizontal coverage,
approximately -90 to +15-degree vertical coverage, 0.1–30 m range and 5 Hz nominal
simulation update rate. It is **not an exact Unitree L1 scan-pattern/noise model**.
There is no artificial plane filling holes or extending beyond the tile boundary.

```bash
ros2 topic hz /sim/terrain/cloud
ros2 topic echo /sim/terrain/cloud --field header --once --qos-reliability best_effort
ros2 run tf2_ros tf2_echo terrain_map terrain_sim_lidar
```

- `/sim/terrain/cloud`: `sensor_msgs/msg/PointCloud2`, metres, best effort.
- `/clock`: Gazebo simulation clock.
- Sensor frame: `terrain_sim_lidar`.
- Level/world reference: `terrain_map`; Z is assumed up in the supplied STL.
- A matching static TF places the scanner at the same position used by Gazebo.

Move the stationary test location by relaunching with measured world coordinates:

```bash
ros2 launch lidar_terrain_description gazebo.launch.py \
  sensor_x:=5.0 sensor_y:=0.0 sensor_z:=2.0
```

`sensor_z` is absolute elevation in terrain_map, not automatic height above the
local ground. Check it is above the surface at the selected XY. Do not drag this
scanner with Gazebo's transform tool: the fixed ROS TF would become incorrect.
`gui:=false rviz:=false` omits both viewers, but GPU LiDAR still requires a working
rendering context. `mesh:=overview` is also available for larger scenes; the
simplified overview is intended for scene exploration, not fine roughness tests.

## Units and origin — verify before quantitative use

STL has no units, CRS, geolocation, scan timestamps or sensor calibration. Nothing
in this file proves it represents the URC/MDRS site or establishes LiDAR provenance.
The default assumes one source unit is one metre. In raw coordinates its bounds
span about 1230.91 x 798.84 x 25.91 units. Both launches accept `scale:=0.001` if
you confirm the export was in millimetres, or another positive uniform scale.
Do not rescale based on appearance alone. Slope survives uniform scaling, but
heights, roughness, distances and appropriate grid resolution change.

The bundled meshes retain source X/Y. Source Z is shifted by subtracting
**-0.8499462008476257** so the median surface near XY=(0,0) lies near Z=0.
The launch scale applies after this translation. Sensor coordinates are always
in metres in the resulting world and are not multiplied by mesh scale.
The default sensor is at (0,0,1.2); it is a test placement, not a measured L1 mount.
Full details and the source SHA-256 are in `meshes/provenance.json`.

## Use with terrain_geometry

Keep the real L1 driver stopped for this simulation session and avoid another
publisher of `/clock` or conflicting terrain_map transforms. Configure your
terrain processor with:

| Setting | Value |
|---|---|
| input_topic | /sim/terrain/cloud |
| target_frame | terrain_map |
| use_sim_time | true |
| Ground seed | Near XY=(0,0), Z approximately 0 for the default scene |

Do not pass a real mounting YAML: Gazebo launch already publishes the simulated
sensor TF. Use the terrain processor's simulation-capable entry point; the older
physical `terrain.launch.py` forces wall time and is unsuitable here. In versions
that expose the `terrain_node` executable, it can be started directly:

```bash
ros2 run terrain_geometry terrain_node --ros-args \
  -p use_sim_time:=true \
  -p input_topic:=/sim/terrain/cloud \
  -p target_frame:=terrain_map
```

That command uses the processor's default grid/seed/classification parameters;
pass your simulation YAML with `--params-file /absolute/path/to/config.yaml` when
needed. Inspect the seed region before treating it as ground. Occlusion and absent
returns must remain unknown. The STL surface has already been reconstructed;
its roughness is not a substitute for real sensor validation.

If using the previously supplied Nav2 adapter, match its clock:

```bash
ros2 launch terrain_geometry nav2_adapter.launch.py use_sim_time:=true
```

This environment provides no rover odometry or motion control. Your Nav2 robot
still needs its own TF tree, footprint, localization and obstacle inputs.

## Mesh preparation and limitations

The uploaded STL is 438,384,434 bytes with 8,767,687 triangles. To make this
package practical, it contains:

- `terrain_tile.stl`: 31,578 original triangles fully inside [-30,30] source XY,
  with the documented Z translation. No holes are filled. Boundary is irregular.
- `terrain_overview.stl`: 199,999 triangles produced by quadric edge collapse;
  small features and fine roughness can change. Used as both visual and collision
  surface when overview mode is selected.

The large original attachment is unchanged and is not duplicated in this ZIP.
`tools/prepare_mesh.py` reproduces both assets with NumPy, trimesh and
fast-simplification; those are offline preparation dependencies, not runtime
requirements. Source mesh rights remain those of the uploaded asset; no new mesh
license or geographic provenance is asserted.

Validation and runtime limits are in `docs/VALIDATION.md`. The screenshot in
`docs/terrain_preview.png` is an offline rendering, not a Gazebo/RViz recording.

References: [Gazebo Harmonic sensors](https://gazebosim.org/docs/harmonic/sensors/),
[Gazebo mesh resources](https://gazebosim.org/docs/harmonic/ros_gz_project_template_guide/).

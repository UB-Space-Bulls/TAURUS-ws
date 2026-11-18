# View the area STL and rainbow terrain geometry in RViz

This guide uses the moving-rover simulation and the
[terrain inspection preset](../../rover_navigation/config/terrain_inspection.rviz).
It displays the supplied area STL together with ground height, slope, and
roughness computed by `terrain_geometry` from the simulated LiDAR.

The STL is the scene background. The rainbow grid contains fitted ground cells
from the currently observed terrain window. The processor does not color the
entire STL or fill unseen ground.

## 1. Build and select the test environment

From the workspace, after ROS/Nav2 dependencies are installed:

```bash
cd ~/DEIMOS-ws/rover_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select terrain_geometry rover_navigation
source install/setup.bash
export ROS_DOMAIN_ID=185
export GZ_PARTITION=rover_full_scope
```

Use the same environment in other terminals. The complete dependency setup and
moving-rover tests are in
[rover_navigation/README_FULL_STACK.md](../../rover_navigation/README_FULL_STACK.md).

## 2. Start with the terrain view

If this simulation is not already running:

```bash
ros2 launch rover_navigation terrain_rover.launch.py rviz_view:=terrain
```

This single launch starts Gazebo, the rover and its sensors, terrain processing,
the terrain-to-costmap adapter, Nav2, the mesh marker, and RViz. Wait for sensor
data and fresh terrain observations before expecting rainbow cells to appear.
The rover remains stationary until a drive command or navigation goal is sent.

To run without the Gazebo window while keeping RViz:

```bash
ros2 launch rover_navigation terrain_rover.launch.py \
  rviz_view:=terrain gui:=false headless_rendering:=true
```

Do not start a second simulation just to change the RViz view.

## 3. Change an existing RViz window

When the simulation is already running:

1. In RViz, select **File > Open Config**.
2. Open this source file:

   ```text
   ~/DEIMOS-ws/rover_ws/src/rover_navigation/config/terrain_inspection.rviz
   ```

   The installed copy is also available at:

   ```text
   ~/DEIMOS-ws/rover_ws/install/rover_navigation/share/rover_navigation/config/terrain_inspection.rviz
   ```

3. Keep **Global Options > Fixed Frame** set to `odom` for the moving simulation.
4. Wait for **STL terrain** and **Terrain geometry - rainbow height (m)**.

Opening this file changes the viewer configuration; it does not restart Gazebo
or send a navigation goal.

## 4. Choose the terrain measurement

Use the checkboxes in RViz's **Displays** panel. Enable one rainbow grid at a
time so overlapping colors remain readable.

| Display | What it shows | Color range |
| --- | --- | --- |
| STL terrain | The supplied mesh on `/terrain_scene/mesh` | Mesh's brown surface |
| Terrain geometry - rainbow height (m) | Fitted ground-cell height, field `z` | Automatic current-window minimum/maximum |
| Terrain geometry - rainbow slope (deg) | Ground inclination, field `slope_deg` | 0–20 degrees |
| Terrain geometry - rainbow roughness (m) | Ground RMS roughness, field `roughness_m` | 0–0.06 m |
| Ground returns - rainbow height (optional) | Measured ground points on `/terrain/ground_points` | Automatic height range |
| Detected obstacle returns (red) | Points classified as obstacles on `/terrain/obstacle_points` | Solid red |
| Raw LiDAR returns (optional) | Input cloud on `/rover/lidar/points` | Height coloring |

The three fitted-grid displays subscribe to `/terrain/grid_points`. That cloud
contains `x`, `y`, `z`, `slope_deg`, and `roughness_m`. Grid cells render as small
spheres so they are visible against the STL surface. These are processed terrain
measurements; the optional raw returns are the input sensor data.

Height is measured in metres in `odom`, not height above the rover. Its rainbow
range automatically changes with the visible data: the same color can represent
a different elevation after the window moves. For consistent comparisons, turn
off **Autocompute Intensity Bounds** in the height display and enter fixed
**Min Intensity** and **Max Intensity** values.

Slope and roughness use fixed display ranges. These color limits do not change
the terrain classifier or Nav2 cost policy. They are demonstration values rather
than measured rover limits. Missing ground remains unknown and has no fitted
rainbow cell.

## 5. Camera views and costmap overlays

The **Views** panel has two saved views:

- **Processed terrain close-up:** inspect cells and their alignment near the rover.
- **Full STL tile:** zoom out to inspect the whole supplied area.

Select a saved view and use **Load** in the Views panel. The camera follows
`base_link`; the fixed reference frame remains `odom`. You can also orbit, pan,
and zoom with RViz's camera controls.

The inspection preset keeps **Terrain costs**, **Global costmap**, and **Local
costmap** disabled initially so their colors do not cover the rainbow geometry.
Enable one of them when comparing geometry with navigation costs, and disable
the rainbow grid if the overlay becomes hard to read.

For the close top-down navigation view, load
[terrain_rover.rviz](../../rover_navigation/config/terrain_rover.rviz), or select
it on the next launch:

```bash
ros2 launch rover_navigation terrain_rover.launch.py rviz_view:=navigation
```

The visible STL extends beyond the processed 12 x 12 m terrain window. Seeing a
surface in this inspection view does not make it a valid navigation goal. Start
with observed, low-cost cells one or two metres from the rover. Use **Nav2 Goal**
for one destination, and leave Nav2 active between goals.

## 6. Check missing or incorrect displays

Run these checks in a terminal with the same workspace and ROS domain sourced:

```bash
ros2 topic info /terrain_scene/mesh --verbose
ros2 topic info /terrain/grid_points --verbose
ros2 topic echo /terrain/grid_points --field header --once
ros2 topic echo /terrain/grid_points --field fields --once
ros2 topic echo /terrain/cost_grid_valid --once
ros2 run tf2_ros tf2_echo odom terrain_lidar_link
```

Stop `tf2_echo` with Ctrl+C. To measure the geometry update rate:

```bash
ros2 topic hz /terrain/grid_points
```

The moving simulation processes terrain at a configured 1 Hz. Actual reception
rate depends on simulation speed and processing load.

| Symptom | What to check |
| --- | --- |
| Only the STL appears | `/terrain/grid_points` must contain valid ground cells; check the terrain node's input, TF, timestamps, and ground seed. |
| Only rainbow cells appear | Enable STL terrain; check `/terrain_scene/mesh` and the installed `lidar_terrain_description` mesh assets. |
| No TF / wrong alignment | Use `odom` for this simulation; confirm both sensor frames and dynamic `odom -> base_link`. Do not add a static base transform. |
| No data in either display | Confirm the full simulation is running and the terminal/RViz use its ROS domain. Loading a preset alone does not start publishers. |
| Rainbow grid is one color | The selected measurement may have a small range, or fixed bounds may be too wide. Inspect the field and bounds before changing classifier settings. |
| Colors overlap or look flickery | Enable one rainbow grid and one costmap overlay at most; use the close-up camera view. |
| Data appears briefly then stops | Check source timestamps, simulation time, processing warnings, and terrain validity. |
| QoS warning | Use the profiles below; reliability/durability must match the topic's publisher. |

Expected subscription settings in this preset:

- STL marker: **Reliable**, **Transient Local**, depth 1.
- Processed grid, ground, and obstacle points: **Reliable**, **Volatile**, depth 1.
- Raw LiDAR: **Best Effort**, **Volatile**, depth 1.

For physical sensing or another point-cloud source, select its actual
gravity-aligned target frame and valid sensor TF. This preset's `odom` and rover
sensor topics describe the combined Gazebo test; they do not provide physical
mount calibration or localization.

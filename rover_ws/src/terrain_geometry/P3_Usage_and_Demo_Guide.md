# P3 — Terrain Geometry: Quick Usage and Demonstration Guide

**Owner:** Devan Ng · **Environment:** ROS 2 Jazzy / Gazebo Harmonic · **Updated:** October 28, 2026

The `terrain_geometry` package processes a stationary simulated Unitree L1 cloud into ground height, slope, roughness and unknown cells, plus separated ground/obstacle points. The live demo has passed its geometry checks on Devan’s machine. A short recording and agreement with the Nav2 members remain to be completed.

## 1. Setup and startup

Use Ubuntu 24.04 with ROS 2 Jazzy, Gazebo Harmonic, colcon and rosdep. Place the single package at `rover_ws/src/terrain_geometry`; it includes its own `msg/TerrainGrid.msg`, launch/config files, meshes, worlds and RViz setup. Use the working source version containing the finite-point filter added during debugging.

For the current workspace:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/DEIMOS-ws/rover_ws

# First setup only:
rosdep update
rosdep install --from-paths src/terrain_geometry --ignore-src -r -y --rosdistro jazzy

colcon build --symlink-install --packages-select terrain_geometry &&
source install/setup.bash &&
ros2 launch terrain_geometry demo.launch.py
```

For later launches, source ROS and `install/setup.bash`, then run the launch command. Source both environments in every additional ROS terminal. Stop with **Ctrl+C**. Use `l1_variant:=RM` to select the RM simulation range; PM is the default.

The launch starts Gazebo, the ROS bridge, sensor transforms, terrain processing and RViz. The sensor is fixed at `(0, 0, 1.2)` m with roll = 180°, facing down into the terrain. `terrain_map` is the level world frame, with +Z upward and the floor at Z = 0. Distances are metres and rotations are radians. Slope uses this level reference; no FAST-LIO, Point-LIO or IMU attitude estimator is needed.

The L1 model uses a 240×90 raster at 1 Hz. It approximates L1 coverage and point budget, not the physical sensor’s nonrepeating scan trajectory. The standalone scene does not spawn the team’s rover; attaching this pipeline to its rover simulation is separate integration work.

## 2. Inspect the four test areas in RViz

The world contains actual surface geometry: a flat floor, a **15° ramp**, an uneven mesh and a **0.7 m raised box**. Set RViz’s **Fixed Frame** to `terrain_map`.

| View | Enable | What to demonstrate |
|---|---|---|
| Ground separation | Ground points, Obstacle points, Unknown ground | Green ground, red raised-obstacle evidence, gray unknown ground |
| Height | Height grid only, optionally Unknown ground | Floor near 0 m and increasing elevation up the ramp |
| Slope | Slope grid only, optionally Unknown ground | Flat terrain near 0° and ramp near 15° |
| Roughness | Roughness grid only, optionally Unknown ground | Uneven patch has higher residual roughness than flat terrain |

Disable other grid layers and point displays when viewing a single property. Grid colors represent numeric values; red grid cells do not necessarily mean obstacles. Yellow “Unclassified returns” are optional and indicate points without a reliable classification. Gray areas can contain sparse returns; they do not imply that Gazebo has no physical surface there.

Keep the sensor stationary for this demonstration. Moving it in Gazebo alone leaves ROS’s fixed transform incorrect. For another fixed viewpoint, change both the sensor model pose and the matching static transform, then restart. Continuous movement requires timestamped pose/TF integration.

## 3. Validate the live output

In a second sourced terminal:

```bash
ros2 topic hz /unilidar/cloud
# Expect roughly 1 Hz; Ctrl+C stops only this measurement.
ros2 run tf2_ros tf2_echo terrain_map unilidar_lidar
# Confirm the mounting transform; Ctrl+C when finished.
ros2 run terrain_geometry check_demo --seconds 20
```

The checker requires at least five distinct snapshots. Every snapshot in its final window of up to ten must pass: flat slope below 2°, ramp within 2° of 15°, roughness at least 8 mm above flat, at least two box obstacle cells, and unknown cells retained. Each measurement region needs at least six known cells.

**Reported live run, October 28:** 19 distinct snapshots received; `passed: true`. The final ten snapshots showed:

| Check | Measured result |
|---|---|
| Flat slope | 0.067–0.097° |
| Ramp slope | 14.916–15.023° against 15° ground truth |
| Flat roughness | 9.85–10.54 mm |
| Uneven-ground roughness | 24.01–26.46 mm |
| Box obstacle evidence | 10 cells per snapshot |
| Cells without valid ground | 1,328–1,337 of 2,000 |

These are live simulation results supplied by Devan, not physical L1 measurements. The checker validates selected regions; inspect the whole RViz scene as well.

## 4. Save the submission evidence

Keep the demo running. From the workspace root in a sourced terminal:

```bash
evidence_run="p3_evidence_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$evidence_run"

ros2 run terrain_geometry check_demo --seconds 20 \
  > "$evidence_run/terrain_live_checks.json"
cat "$evidence_run/terrain_live_checks.json"

bash "$(ros2 pkg prefix --share terrain_geometry)/scripts/record_demo.sh" \
  "$evidence_run/terrain_demo_bag" 60
ros2 bag info "$evidence_run/terrain_demo_bag" \
  > "$evidence_run/bag_info.txt"

tar --exclude='__pycache__' -czf "$evidence_run/terrain_geometry_source.tar.gz" \
  -C src terrain_geometry
```

Also save a **45–60 second screen recording** in the evidence folder. Show the four Gazebo surfaces, green/red ground separation, height, slope and roughness individually, gray unknown areas, and the checker’s passing result. A rosbag is supplementary data, not a substitute for the video. Verify the video plays and bag metadata lists `/unilidar/cloud`, `/terrain/grid`, `/tf_static` and `/clock`.

The source archive preserves the actual tested code, including local fixes. Package files include `launch/`, `config/`, `worlds/`, `meshes/`, `msg/` and `docs/`.

## 5. Nav2 handoff

Publish/subscribe to **`/terrain/grid`**, type **`terrain_geometry/msg/TerrainGrid`**. Share `msg/TerrainGrid.msg` and `docs/NAV2_GRID_CONTRACT.md` with the Nav2 members.

| Property | Contract |
|---|---|
| Frame | `header.frame_id`; `terrain_map` in this demo, gravity aligned |
| Geometry | 50×40 cells; 0.3 m resolution; origin `(-6,-6,0)` m at the lower corner; identity orientation |
| Indexing | Row-major: `index = row_y * width + column_x`; each array has 2,000 entries at default settings |
| Timestamp | Input cloud acquisition stamp; simulation time in Gazebo |
| Updates / QoS | Complete replacement snapshot, nominally 1 Hz; reliable, volatile, keep-last depth 1 |
| `ground_height` | Ground elevation at the cell center, metres |
| `slope` | Inclination from horizontal, radians; only the visualization field `slope_deg` uses degrees |
| `roughness` | RMS perpendicular ground-point distance from the fitted local plane, metres |
| `status` | 0 unknown; 1 valid ground; 2 obstacle evidence with unknown ground; 3 ground plus obstacle evidence |
| Unknown values | Height/slope/roughness are NaN without valid ground; zero is a valid measurement |
| Counts | `ground_count` and `obstacle_count` count classified observations; zero obstacles does not prove clearance |

The adapter must prioritize obstacle evidence, preserve unknown cells, reject stale snapshots and transform/resample if its costmap frame differs. Agree on frame, resolution, origin, timestamp handling, expiry and slope/roughness thresholds before integration. A 2-second acquisition-age limit is proposed, not yet agreed. `/terrain/costmap_preview` is diagnostic only; this package does not implement a Nav2 costmap layer.

## 6. Method, limits and completion

The method selects lower-height candidates in grid cells, grows ground from observed seeds near a known height, and fits robust local planes. Plane inclination gives slope; residuals give roughness. Points sufficiently above nearby valid ground are obstacle evidence. Method selection and references are documented in `docs/METHOD.md`.

Missing returns remain unknown. The method does not establish reliable ditch detection or recognize grass, sand or mud. Moving-map accumulation is deferred until reliable pose estimates are available. Physical L1 testing is pending: use the separately installed vendor driver, measured mounting TF and appropriate ground-seed settings; `docs/UNITREE_L1.md` describes the hardware launch path.

Before final submission:

- [x] Live build, simulated cloud delivery and terrain checks demonstrated.
- [x] Grid fields, separated points, launch/configuration files and test geometry implemented.
- [ ] Save and review the short video, live JSON, bag metadata and tested source archive.
- [ ] Resolve the remaining SDF noise-format warning and rerun checks for the final recorded version: LiDAR noise requires `<noise><type>gaussian</type>...</noise>`.
- [ ] Confirm the grid contract with the Nav2 members; send them the message definition and contract.
- [ ] Test rover-mounted simulation and physical L1 data when available.

If a symlink build fails, move only this package’s old `build/terrain_geometry` and `install/terrain_geometry` outputs to a backup, rebuild, and launch only after success. Keep the finite XYZ filter before transforming clouds; missing rays can include infinities. The earlier Fast CDR crash was resolved by updating the mismatched ROS middleware packages.

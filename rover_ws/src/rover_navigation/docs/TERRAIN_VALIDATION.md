# Terrain/Nav2 runtime validation — 17 November 2026

Source: `nav2` commit `730c412f3fab6b3c654cf606098fa659fa3241e7`.
Host: ROS 2 Jazzy, Ubuntu 24.04, Gazebo Harmonic, Nav2 1.3.14.

## Passed

- Full workspace build: all six packages.
- `rosdep check`: all system dependencies satisfied for navigation, terrain,
  and the LiDAR terrain scene.
- Terrain tests: 47 passed.
- Navigation package lint checks: nine reported tests, no errors or failures.
- Clean start of `terrain_bench.launch.py` with viewers disabled and headless
  rendering enabled, on ROS domain 184 and a dedicated Gazebo partition.
- Controller, planner, behavior server, velocity smoother, collision monitor,
  and BT navigator all reached lifecycle state `active`.
- All 11 checked processing/navigation nodes and costmaps use simulation time.
- Valid TF connects terrain_map, terrain_sim_lidar, odom, and base_link;
  lookup succeeds at the cloud stamp. The sensor is at Z=1.2 m.
- Exactly one `/terrain_cost_grid` publisher: `terrain_cost_adapter`, using
  reliable/transient-local QoS. Both costmaps subscribe to it.
- Navfn computed a 4.935 m path containing 196 poses between observed cells
  near (-2.475, -0.175) and (2.475, -0.175), without commanding motion.
- Pausing the simulator expired the cost grid to all unknown and set validity
  false after 2.32 seconds of wall time. Costs recovered after resuming.

## Measured scan and costmap data

| Output | Result |
| --- | --- |
| Finite 3D points per scan | 38,547 |
| Cloud frame | terrain_sim_lidar |
| Received cloud rate during the final short check | 4.05 Hz wall / 4.17 Hz simulation |
| Horizontal scan bins | 720; 111 finite returns; no NaNs |
| Terrain cost input | 936 known / 664 unknown / 258 graded cells |
| Local costmap | 14,286 known / 114 unknown / 1,295 graded cells |
| Global costmap | 33,696 known / 126,304 unknown / 9,288 graded cells |
| Source-unknown cells sampled in the local/global costmaps | 3 / 533; all remained unknown |

The LiDAR is configured for 5 Hz. These short-run reception rates include
startup and CPU/GPU load and are not a sustained performance guarantee.
Finite XYZ coordinates were checked for spatial spread and sensor-range bounds;
publication rate alone was not used to claim valid scans.

The detailed final report is `log/terrain_nav2_validation.json` in the workspace;
the launch log is `log/terrain_nav2_bench.log`. The checker is installed as
`ros2 run rover_navigation check_terrain_integration.py`. Test processes were
stopped after validation; shutdown interrupt messages can appear in the log.

## Issues resolved

Nav2 was initially absent. After installation, lifecycle_manager failed with
an undefined diagnostic_updater constructor symbol: diagnostic_updater 4.2.6
was incompatible with the newly installed Nav2 build. Upgrading that dependency
to 4.2.7 resolved the failure. Both the cold start and planner checks then passed.

The original branch supplies synthetic terrain and adds terrain costs only to
the global costmap. The new perception profile uses measured terrain costs in
both maps and preserves unknown space. A point-cloud-to-scan converter supplies
the `/scan` expected by the branch's obstacle layers and collision monitor.
Optional RViz, headless Gazebo rendering, and a bench-specific RViz config allow
both unattended testing and interactive inspection.

## Limits

This validates a stationary simulation with known test transforms and synthetic
zero-motion odometry. It does not validate moving-rover localization, motor
commands, goal execution, or physical terrain classification. No physical L1
serial device was connected, so hardware LiDAR scans were not tested.

The adapter expires upstream data while it remains alive. Nav2 StaticLayer can
retain its last map if the adapter itself stops; a moving-rover supervisor still
needs to handle an absent/false validity heartbeat. Ground uncertainty remains
unknown rather than being cleared to make the planner accept a start pose.

EGL and SDF `gz_frame_id` warnings occurred during simulation startup. They did
not prevent finite clouds, matching ROS frame IDs, or successful costmap and
planner checks. Viewers were disabled during the final run, so this is not an
RViz screenshot or a visual-quality validation.

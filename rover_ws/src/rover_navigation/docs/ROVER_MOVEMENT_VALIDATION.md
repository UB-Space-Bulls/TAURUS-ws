# Moving rover / terrain / Nav2 validation — 17 November 2026

Run instructions: [README_FULL_STACK.md](../README_FULL_STACK.md).

## Scope and implementation

`terrain_rover.launch.py` now combines the driving rover, the supplied STL
terrain collision surface, a rover-mounted 3D GPU LiDAR, the original horizontal
scanner, Gazebo 3D odometry, the rolling terrain processor and cost adapter,
both terrain-aware Nav2 costmaps, the command chain, and optional viewers.
It uses one simulator, one clock, one dynamic odometry tree, and one terrain
cost publisher. The stationary fixture and synthetic cost publisher are absent.

The rover model's new terrain sensor and 3D odometry are opt-in Xacro arguments.
The original launch defaults retain the original 2D sensor/odometry setup.
Gazebo's odometry dimensions and world pose semantics follow the upstream
[OdometryPublisher implementation](https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/odometry_publisher/OdometryPublisher.cc).

The moving simulation profile enables clearing only the current occupied
footprint in Nav2. The raw measured grid retains unknown space, and the planner
rejects unknown terrain. A configurable 0.65 m self filter discards rover-body
returns before terrain fitting. It does not label the discarded area free.
The existing stationary/physical profile remains conservative with footprint
clearing disabled and the new self filter disabled by default.

The terrain-validity command gate uses wall time and rejects false/absent
validity or old velocity commands. The swerve node independently stops wheel
commands after 0.5 seconds without `/cmd_vel`. These controls address a retained
StaticLayer map when the adapter dies or loses upstream sensing; they are
simulation protections rather than validated hardware supervision.

## Final clean-start runtime result: PASS

Environment: Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, Nav2 1.3.14,
ros2_control 4.50.0, gz_ros2_control 1.2.20, diagnostic_updater 4.2.7.
Viewers were disabled; Ogre2 headless rendering supplied actual LiDAR returns.
ROS domain 185 and a dedicated Gazebo partition isolated the test.

```bash
ros2 launch rover_navigation terrain_rover.launch.py \
  gui:=false rviz:=false headless_rendering:=true
ros2 run rover_navigation test_rover_navigation.py \
  --drive --navigate --goal-distance 1.5 --pause-test \
  --report log/terrain_rover_validation.json
```

The commands run in separate terminals with the same ROS/Gazebo environment.
The verification wrapper stopped its simulator after testing.

| Check | Measured result |
| --- | --- |
| Joint controllers | joint_state_broadcaster, steer_controller, wheel_controller active |
| Nav2 lifecycle | all six managed nodes active |
| Mounted 3D LiDAR | 38,428 finite points; frame terrain_lidar_link |
| Horizontal scan | at least 300 bins; frame lidar_link |
| TF | dynamic odom -> base_link; both sensor frames connected |
| Terrain input | 942 known, 658 unknown, 243 graded cells |
| Local costmap | 14,400 known, 0 unknown, 1,194 graded cells |
| Global costmap | 34,272 known, 125,728 unknown, 8,748 graded cells |
| Manual forward drive | 0.262 m odometry displacement |
| Relative Nav2 goal | requested 1.5 m in the current heading |
| NavigateToPose result | SUCCEEDED (status 4), error_code 0 |
| Motion during navigation | 1.254 m displacement |
| Final goal error | 0.246 m; configured goal tolerance 0.25 m |
| Initial/final base XY | (-0.0057, 0.0108) -> (1.5202, 0.0108) m |
| Rolling grid origin X | -6.3 -> -4.8 m |
| Maximum wheel command | 2.019 rad/s |
| Terrain loss/paused clock | validity false; terrain grid all unknown |
| Nonzero input while invalid | gate and wheel commands stayed zero |
| Resume | fresh terrain and validity recovered |

The small local window was fully observed outside the supported footprint;
zero local unknown cells is acceptable in this scene. The larger global
window and the raw terrain grid retained unknown areas.

The full report is `log/terrain_rover_validation.json` in the workspace. The
final launch log is `log/terrain_rover_final_launch.log`. Unit and lint results
are in each package's `build/<package>/test_results` directory.

## Source checks

- Terrain package: 47 tests passed.
- Navigation package: motion-guard regressions and lint checks passed, with
  17 checks reported by colcon test-result before adding this documentation.
- Regression cases cover command-loss stopping with steering held, fresh
  terrain/commands passing velocity, false or absent heartbeat blocking fresh
  commands, and expired commands blocking despite healthy terrain.
- Default and terrain-enabled rover Xacro modes expand and parse; the default
  has no extra scanner and keeps 2D odometry, while terrain mode has the extra
  scanner and 3D odometry.
- All declared dependencies for rover, perception, and navigation are satisfied.

## Problems found during integration

The older controller_manager binary could not load after diagnostic_updater was
updated for the newer Nav2 build. Updating the controller/hardware interfaces,
manager/messages, controllers, realtime tools, and Gazebo control plugin together
resolved the ABI mismatch. The README records the package update command.

Initial driving exposed body returns being classified as terrain obstacles.
The optional self filter removes those points from classification. Short
collision-ahead rejections can still occur while a rolling grid catches up
with motion; the tested Nav2 action recovered and completed. Collision checking
and unknown rejection remained enabled throughout the successful test.

EGL, SDF sensor extension, root-link inertia, and control update-rate warnings
appeared during startup but did not prevent controllers, scans, terrain costs,
or navigation. Shutdown can require Gazebo to receive SIGTERM after its SIGINT
timeout; this is separate from navigation success. A shutdown-only steering-node
traceback was corrected after the successful motion run.

## Remaining acceptance work

Interactive goal-abort follow-up on the same date found five destinations at
approximately 14–25 m from the origin; planner logs explicitly reported
`outside bounds`. Empty NavigateThroughPoses requests reported
`No viapoints given`. Panel resets then left all six Nav2 nodes unconfigured,
which explained RViz's unavailable-action-server messages. A lifecycle startup
restored the nodes. A new relative 1.5 m goal in the user's running simulation
succeeded with 1.255 m displacement and 0.245 m final error. Its report is
`log/nav2_goal_abort_fix_validation.json`.

The default RViz view now follows the rover in a close top-down view with a
1 m grid. Full-scene mesh and raw-cloud overlays are optional so a visible
surface outside processed terrain is not mistaken for a valid goal. The test
checker now waits for a fresh graded snapshot after lifecycle checks, avoiding
a false failure if those checks overlap the adapter's expiry/refresh cycle.

A subsequent viewer-startup check found that the nested Nav2 launch's
`rviz:=false` was leaking into the parent launch. Scoped GroupAction wrappers
now preserve the parent's viewer setting; regressions verify both true and
false settings, and the navigation package reports 20 passing checks. RViz was
also started with the full-stack configuration and OpenGL 4.6; its plugins
loaded without a startup failure. This checks window startup rather than a
visual acceptance of the complete live scene. A leftover test Gazebo server
sharing the user's ROS domain was stopped to resolve controller-spawner conflicts.

The optional box-obstacle and unknown-goal scenarios in the README are manual
acceptance procedures; they are not included in the measured short-goal result.
RViz and Gazebo GUI rendering were not visually inspected in the final run.
Long routes, repeated goals, steep/rough limits, localization drift, drop-offs,
and sustained real-time performance need their own measurements.

No physical L1 or hardware motor test was performed. Ground-truth Gazebo
odometry is not SLAM, and the supported-footprint assumption, provisional CAD
geometry, seed height, terrain units, and generic sensor model require review
before any physical deployment.

# FAST-LIO source import

Imported from the versions pinned by the team's
[TAURUS-ws localization/fast-lio branch](https://github.com/UB-Space-Bulls/TAURUS-ws/tree/localization/fast-lio/rover_ws/src),
commit `7295ce36432453ebddb38e30611daf3bf9fc75f1`. The fetched DEIMOS remote branch
and the linked TAURUS branch resolve to this same commit:

| Package | Repository | Revision |
| --- | --- | --- |
| fast_lio | https://github.com/Ericsii/FAST_LIO_ROS2 | `2fffc570a25d0df172720bac034fbdb6a13d2162` |
| livox_ros_driver2 | https://github.com/Ericsii/livox_ros_driver2 | `378d1c7c9d33ec44d0683aef885b6ed0cce9612c` |
| ikd-Tree submodule | https://github.com/hku-mars/ikd-Tree | `e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4` |

The Livox package supplies messages and libraries required by this FAST-LIO
source. Importing/building it does not start a Livox hardware driver or replace
`unitree_l1_driver`.

The FAST-LIO checkout now has a local one-line IMU subscription QoS patch for
the L1's Best Effort publisher. The base revisions above remain pinned. Apply
`patches/fast_lio-imu-sensor-qos.patch` after a fresh import to reproduce this
change; see the physical test guide below.

## Reproduce the import

From a clean workspace without these repositories already present:

```bash
cd ~/DEIMOS-ws/rover_ws
vcs import --recursive . < fast_lio.repos
```

The two source directories are independent Git checkouts. Keep `fast_lio.repos`
with the workspace to reproduce their revisions; their parent repository does
not automatically include their contents. Importing these packages did not
merge the localization branch or replace the rover/terrain/navigation sources.

## Build

```bash
source /opt/ros/jazzy/setup.bash
cd ~/DEIMOS-ws/rover_ws
sudo apt-get install ros-jazzy-pcl-ros libapr1-dev libaprutil1-dev
rosdep check --from-paths src/fast_lio src/livox_ros_driver2 \
  --ignore-src --rosdistro jazzy
PYTHONNOUSERSITE=1 CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 \
  colcon build --packages-up-to fast_lio --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
ros2 pkg executables fast_lio
ros2 launch fast_lio mapping.launch.py --show-args
```

The import build completed successfully for both packages on ROS 2 Jazzy.
`ros2 pkg executables fast_lio` finds `fastlio_mapping`, launch argument discovery
succeeds, and the installed mapping executable has no unresolved shared libraries.
Upstream CMake/deprecation warnings remain. Third-party test suites were disabled
for this build; these checks do not establish live sensor operation or odometry
accuracy.

## Comparison with the localization branch

| Component | Current terrain/navigation workspace |
| --- | --- |
| FAST-LIO estimator and Livox dependency | Imported at the exact revisions in the branch's `third_party.repos`. |
| `rover_localization` | Not imported. Its `fast_lio_test.launch.py`, Gazebo config, comparator and RViz setup are not part of the current launch. |
| `rover_description` | Existing terrain/navigation version retained. The linked branch's sensor/bridge changes were not imported. |

The branch's localization test includes `rover_description/gazebo.launch.py`
and FAST-LIO's `mapping.launch.py`, configured for `/lidar/points` and `/imu`.
Its rover description supplies a 32-layer simulated LiDAR and a 200 Hz IMU.
The current terrain rover instead supplies `/rover/lidar/points` plus `/scan`
and has no simulated IMU. Copying the branch's launch alone would therefore
leave FAST-LIO without its expected inputs.

## What remains to connect

The current `terrain_rover.launch.py` still uses Gazebo ground-truth odometry.
It does not start FAST-LIO. The current rover model also needs an appropriate
IMU source for a complete LiDAR-inertial test; adding the estimator alone is
insufficient. The localization branch's bringup package has not been imported.

FAST-LIO's upstream default launch/config targets Livox hardware. For the L1,
the optional `unitree_l1_driver/l1_fastlio.launch.py` uses the supplied L1
starting config, SDK-time profile, timed type-2 cloud layout and patched IMU QoS.
See [FAST_LIO_L1_TESTING.md](src/unitree_l1_driver/docs/FAST_LIO_L1_TESTING.md)
for commands, clock checks and hardware validation limits. The SDK measurement
timing and odometry accuracy still need physical verification. Its registered
cloud and body TF can then feed the existing terrain adapter described in
[FASTLIO_INTEGRATION.md](src/terrain_geometry_physical/terrain_geometry/docs/FASTLIO_INTEGRATION.md).
Nav2 also requires a deliberately defined frame relationship and one TF
authority per edge. The existing adapter does not make those connections by
itself.

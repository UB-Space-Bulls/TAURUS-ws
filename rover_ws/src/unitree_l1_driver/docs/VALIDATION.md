# Validation of the two-package physical edition

Prepared 12 November 2026. These are software checks in a Linux x86_64 runtime,
not a physical L1 demonstration.

| Check | Result |
|---|---|
| Native SDK and passive wire readers: CMake build/link with supplied x86_64 SDK | Passed |
| Driver health and synthetic serial tests | 15 passed, including real SDK decoding of PTY fixtures |
| Terrain geometry, nonfinite filtering, measured mounts and mount-file CLI | 23 passed |
| Python syntax, package XML, YAML/RViz configuration and local guide links | Passed (vendor documentation retained as upstream excerpts) |
| Script installation, normal and upstream Jazzy symlink handler | Passed from source files deliberately set to 0644; installed scripts executable |
| Installed imports/native reader lookup and missing-device report | Passed in staged installations |
| ZIP integrity and executable permissions after unzip | Passed |
| Grid message definition | Byte-identical to the previous terrain package |
| Package boundaries | Separate manifests; neither depends on the other; no Gazebo dependency |
| ROS 2 Jazzy colcon build, live launch and RViz | Not run: ROS 2 absent in this environment |
| Real L1, USB adapter, real terrain and aarch64 execution | Not run: hardware unavailable |

The staged install check uses the actual upstream Jazzy program-install handler
and CMake install rules; it is not a full ROS/ament build.

The terrain tests cover a known 15-degree ramp, flat/rough surfaces, raised
obstacles, insufficient observations, occlusion-like holes, invalid values,
transforms and single-scan independence. The mount helper tests verify that
blank/nonfinite input is reprompted, existing files are protected and cancelled
input leaves no partial YAML.

Driver tests include cloud/IMU decoding through the bundled SDK, deliberate
packet corruption, stalled streams, malformed records and competing readers.
The synthetic fixtures do not establish hardware calibration or power health.
The original SDK buffering workaround is retained; see the driver package's
`docs/PREVIOUS_VALIDATION.md` for its rationale.

Repeat terrain tests from the terrain package root:

```bash
OPENBLAS_NUM_THREADS=1 PYTHONPATH=. python3 -m unittest discover -s test -v
```

Repeat native driver tests from the driver package root, without ROS sourced:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j2
ctest --test-dir build --output-on-failure
```

On the Jazzy computer, also run `colcon test --packages-select unitree_l1_driver
terrain_geometry` and `colcon test-result --verbose` after building. Physical
acceptance still requires positive raw cloud/IMU data, a measured mounting TF,
and comparison with a known floor/ramp/raised obstacle. A short real recording
must be collected there; none is fabricated here.

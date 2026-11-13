# Validation — terrain_geometry v0.5.0

Updated 13 November 2026. ROS 2 is not installed in this environment; the new
Nav2 connection has not been launched against Jazzy or real hardware here.

| Current check | Result |
|---|---|
| Offline terrain, mounting, cost conversion, freshness and profile tests | 38 passed |
| Cost conversion | Unknown preserved; obstacles take priority; graded/lethal thresholds and float32 boundaries checked |
| Freshness | Expiry, paused clocks, repeated/out-of-order stamps and clock resets checked |
| DEIMOS configuration | Generated from nav2 commit 730c412f3fab6b3c654cf606098fa659fa3241e7; both costmaps configured, scan and robot geometry preserved |
| Config CLI | Separate output generated; existing output and original input protected |
| Python syntax, XML, YAML/RViz, internal Markdown links | Passed |
| Archive | Integrity, single package, executable scripts and unchanged TerrainGrid message checked |
| ROS 2 launch, Nav2 lifecycle/TF, RViz and real L1 | Not run; required on the ROS computer |

The implementation was checked against the upstream Jazzy StaticLayer and
CostmapLayer source: full OccupancyGrid input, graded costs, unknown handling and
rolling-frame transforms. This source review does not replace a running Nav2
test. In particular, StaticLayer retains its last map if the adapter dies; the
freshness heartbeat needs independent supervision before autonomous motion.
See [NAV2_INTEGRATION.md](NAV2_INTEGRATION.md) for the bench and integration checks.

The earlier checks below are retained as the prior release record. Native driver
builds and staged ament installations were not rerun for this adapter-only update.

## Earlier two-package edition checks (v0.4.0)

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

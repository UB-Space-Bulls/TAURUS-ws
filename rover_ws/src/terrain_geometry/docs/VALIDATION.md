# Validation status

Geometry results prepared 2026-10-25; the 23 tests were rerun after the
single-package conversion on 2026-10-28. These results are from offline code in the
authoring environment. They are not measurements from a running ROS/Gazebo
system or the physical rover.

| Check | Result |
|---|---|
| Python geometry regression tests | 23 passed |
| Flat region median slope | 0.04484°; 70/70 interior region cells valid |
| Ramp median slope | 15.0321° vs 15.0° ground truth; 17/24 interior cells valid |
| Uneven region median roughness | 0.0216934 m; 23/45 interior cells valid |
| Flat median roughness | 0.0041784 m with synthetic range noise |
| Raised box cells with obstacle evidence | 8 |
| Cells without valid ground | 1318 / 2000, preserved as unknown ground |
| First-return cloud size | 20,532 returns before ROI filtering |
| Measured processing duration | About 0.30 s for this cloud on this runtime; not a rover benchmark |

The offline model uses the same triangle geometry and LiDAR angle/range limits
as the L1-envelope test world (240×90 rays, inverted native hemisphere). It
selects the nearest hit, accounting for occlusion and the finite floor, then
adds synthetic Gaussian range noise (σ=0.01 m, seed 42). These are simulated
measurements, not a claim of physical sensor accuracy. Areas with insufficient
returns and areas behind raised objects remain unknown. The 21,600 rays in a
1 Hz raster are a simulation batching choice, not the real L1 mirror pattern.

Tests cover known planes, roughness, a raised box, mixed ground/obstacle cells,
empty scans, missing returns, absent ground seeds, sparse and collinear support,
negative height steps, boundary/nonfinite filtering, scan independence, rigid
transforms and invalid configuration. Seven additional tests check L1 mounting,
PM/RM ranges, rate-budget semantics, generated worlds, three noise seeds, and
the inability of an upright sensor to see flat ground below it. A negative step remains unclassified;
that test is not a claim of ditch detection.

See `offline_validation.json` for machine-readable results and
`offline_validation.png` for the visualization. Reproduce with
`scripts/validate_offline.py`.

The single-package conversion was also checked for exactly one package manifest,
unchanged grid fields, valid Python/XML/YAML, local mesh and documentation links,
and executable import resolution in staged normal and symlink layouts. Those
entry-point checks reached the expected missing `rclpy` dependency in this
environment; they do not establish a successful ROS build or node startup.

Still required on a machine with ROS 2 Jazzy and Gazebo Harmonic:

- Build the combined ROS package (message generation and Python node) with colcon.
- Run `demo.launch.py`; confirm GPU LiDAR rendering and point-cloud bridge.
- Confirm source units, frame, mounting, clock and transform availability.
- Run the live checker, inspect all RViz layers and capture the requested video.
- For later rover integration, supply a 3D source and measure its stationary TF.
- Review the proposed grid contract with the Nav2 member(s).
- Validate a physical L1 device once its scans and mount calibration exist.

No short live screen recording or rosbag was produced here. The recording
script is supplied for collecting that evidence; no synthetic video is being
presented as a live demonstration.

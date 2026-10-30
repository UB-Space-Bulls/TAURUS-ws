# Method selection

For this first stationary demonstration, use lower-height cell candidates,
connectivity to a known ground patch, and local robust plane fitting. This
keeps the implementation inspectable and independently testable with NumPy.
The sensor now uses the L1 envelope and inverted mount described in
[UNITREE_L1.md](UNITREE_L1.md). The raster timing is a stationary demonstration
approximation. The level reference is supplied by the experiment, not estimated from every
cloud, so a visible ramp cannot redefine the meaning of horizontal.

| Approach considered | Fit for the first demonstration |
|---|---|
| One global plane, including a RANSAC fit | Useful flat-floor baseline, but one plane cannot describe both a ramp and uneven terrain |
| Unconstrained lowest return in every cell | Simple elevation estimate; an isolated roof or box top can become false ground |
| Lower candidate cells + seeded continuity + local planes | Selected: handles connected ramps and local roughness with explicit unknown support |
| Full probabilistic elevation mapping / pose accumulation | Useful later, but introduces pose/covariance and temporal mapping dependencies outside the stationary scope |

The lower-column idea is related to the grid_map_pcl approach, which groups
points into x/y columns and estimates elevation from a selected cluster. This
implementation adds a known-height seed and connectivity check; it does not
claim to implement or benchmark that package. A robust local plane supplies
slope, while residuals from the observed ground points supply roughness.

`grid_map` is a capable multilayer mapping library. The initial handoff instead
uses a small explicit row-major message to avoid importing its buffer/index
conventions into a Python prototype. If the Nav2 team already uses `GridMap`,
agree on a conversion or replace the transport while retaining these field
semantics and tests.

Primary references checked for this implementation:

- [Gazebo Harmonic sensor tutorial](https://gazebosim.org/docs/harmonic/sensors/):
  sensor system, GPU LiDAR and horizontal/vertical scan configuration.
- [ros_gz_bridge, Jazzy branch](https://github.com/gazebosim/ros_gz/blob/jazzy/ros_gz_bridge/README.md):
  PointCloudPacked ↔ PointCloud2 conversion and one-direction YAML bridges.
- [ANYbotics grid_map](https://github.com/ANYbotics/grid_map): multilayer 2.5D maps
  and unknown cells; reference for alternatives, not a runtime dependency.
- [grid_map_pcl method](https://github.com/ANYbotics/grid_map/blob/master/grid_map_pcl/README.md):
  column-based point-cloud elevation extraction.

The earlier repository integration targeted TAURUS-ws commit
`c062f8f934c37313d9630658cab15f38679dfedf`. This single-package edition is
standalone and can be placed in any Jazzy workspace. No TAURUS or lyra source
is required for the stationary demo.

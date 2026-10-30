# Weekly progress tracker — draft

Updated October 28, 2026 for the next Friday update. Replace the reporting
date if using a different week. This draft distinguishes authored code from
live testing; it has not been sent to anyone.

| Tracking field | Team update |
|---|---|
| Friday date and reporting member | October 30, 2026 — Devan (draft) |
| Assigned members | Devan |
| What has been done | Implemented terrain processing and a grid message; added launch/configuration files, a stationary 3D LiDAR world with real ramp/rough/obstacle geometry, RViz views and a proposed Nav2 grid contract. Updated the simulated sensor to an L1 PM/RM envelope with inverted mounting, Unitree topic/frame names and an IMU. Added an optional physical L1 driver launch. Consolidated the processor, message, worlds and launch/configuration files into one drop-in terrain_geometry package; its grid type is now terrain_geometry/msg/TerrainGrid. Twenty-three offline tests pass. A first-return ray-casting check measures the known 15° ramp at about 15.03° with the chosen synthetic range noise, detects the raised box and preserves unknown cells. |
| Future steps | Build/run with ROS 2 Jazzy and Gazebo Harmonic; verify sensor transforms and live metrics; record the RViz/Gazebo demo and rosbag; agree on the grid contract with the Nav2 members; test physical LiDAR scans when available. Moving-map accumulation and material recognition remain outside this first demo. |

# Proposed P3 → Nav2 grid contract, version 1

This is an implemented proposal. Agreement with the Nav2 members is **pending**;
no coordination message has been sent and no Nav2 costmap plugin is included.

The single-package bundle generates this message inside `terrain_geometry`.
Subscribers using the older `terrain_geometry_msgs` type must update their
imports/dependencies and rebuild; see [MIGRATION.md](MIGRATION.md).

| Property | Meaning / default |
|---|---|
| Topic / message | `/terrain/grid`, `terrain_geometry/msg/TerrainGrid` |
| Frame | `header.frame_id`; `terrain_map` in the fixed test stand, gravity aligned |
| Timestamp | Exact input `PointCloud2.header.stamp`, ROS simulation time in Gazebo |
| Snapshot | Full replacement from one scan; no accumulation and no circular buffer |
| Dimensions | `info.width=50`, `info.height=40` cells |
| Resolution | `info.resolution=0.3` m/cell |
| Origin | `info.origin=(-6,-6,0)` m; lower x/y grid corner, identity quaternion |
| Bounds | x=[−6,9), y=[−6,6); upper limits are exclusive |
| Index order | `k = row_y * width + column_x`; x varies fastest |
| Cell center | x=origin.x+(column+0.5)·resolution; y=origin.y+(row+0.5)·resolution |
| Load time | `info.map_load_time` equals acquisition stamp for this snapshot |
| QoS | Publisher reliable, volatile, keep-last depth 1; subscribe reliable or best effort |
| Default update | 1 Hz, newest cloud only; actual rate depends on processing capacity |

Every array has exactly `width*height` entries. The message is a custom simple
row-major grid, **not** `grid_map_msgs/GridMap`; do not apply grid_map circular
buffer indexing or its center-position conventions.

| Array | Unit / interpretation | Unknown behavior |
|---|---|---|
| `status` | Bit 1: GROUND_VALID; bit 2: OBSTACLE_PRESENT | 0 = no reliable ground/obstacle classification |
| `ground_height` | m; fitted local ground elevation at cell center in the header frame | NaN unless bit 1 set |
| `slope` | radians; nonnegative inclination relative to horizontal | NaN unless bit 1 set |
| `roughness` | m; RMS perpendicular residual of accepted cell ground points to the local plane | NaN unless bit 1 set |
| `ground_count` | Number of accepted ground returns in the cell, before support subsampling | 0 when no valid ground |
| `obstacle_count` | Number of raised returns classified using valid nearby ground | 0 means no positive evidence, not proven clear |

Status values: **0 unknown**, **1 observed ground**, **2 obstacle evidence with
unknown underlying ground**, **3 ground and obstacle evidence**. Zero height,
zero slope and zero roughness are valid measurements, not unknown markers.
The gray RViz cubes represent unknown ground, so cells with status 2 remain gray
under their red obstacle returns.

Adapter behavior to agree before use:

1. Verify frame, array lengths, positive resolution and identity grid orientation.
   Reject stale snapshots and timestamps unexpectedly in the future. A proposed
   maximum age is 2 seconds **from acquisition**, with a matching simulation clock;
   make the final threshold explicit with the Nav2 owner. Missing new messages
   must not preserve old free space indefinitely. Clear adapter state on a
   clock reset or changed grid geometry.
2. Obstacle evidence has priority over ground. Map it to the adapter's lethal
   cost, independent of ground validity.
3. Without valid ground, retain Nav2's unknown state. Do not turn NaN, an empty
   cell, missing returns or an extrapolated obstacle reference into free space.
4. With valid ground, convert slope/roughness using thresholds approved for the
   rover. Height itself is not a cost. Apply footprint checks, inflation and any
   required safety margin in navigation. The P3 classification max slope of 35°
   is an algorithm bound, not a safe rover climbing angle.
5. Transform/resample into Nav2's costmap frame if needed; do not merely rename
   `terrain_map` to `map`. Preserve unknown and obstacle evidence conservatively
   when changing resolution. This initial grid has a fixed origin.

`/terrain/costmap_preview` is a diagnostic `nav_msgs/OccupancyGrid`: -1 unknown,
0 observed below demo thresholds, 100 obstacle or threshold exceedance. Its
default thresholds (20° slope, 0.06 m roughness) are illustrative and uncalibrated.
It is not a ready-to-use Nav2 layer and is intentionally not published on `/map`.

Related outputs, all stamped with the same input stamp and expressed in the
target frame:

| Topic | Type / contents |
|---|---|
| `/terrain/ground_points` | PointCloud2, x/y/z only, accepted ground |
| `/terrain/obstacle_points` | PointCloud2, x/y/z only, positively classified raised returns |
| `/terrain/unclassified_points` | PointCloud2, x/y/z only, retained ROI points without a classification |
| `/terrain/grid_points` | PointCloud2 at known cell centers; x/y/z, slope_deg, roughness_m |
| `/terrain/unknown_cells` | Marker CUBE_LIST, missing/invalid ground drawn at reference height |

Original point fields such as ring, intensity and per-point timestamps are not
preserved in the separated visualization clouds. Subscribe to the source cloud
for those fields. Excluded/nonfinite/out-of-ROI points appear in none of the
three separated clouds.

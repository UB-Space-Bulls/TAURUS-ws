# A1 Terrain Cost Grid Interface

## Purpose
Provide traversability costs to the Nav2 global costmap.
Terrain costs influence route selection without requiring
the terrain to be physically impassable.

## ROS 2 Interface
- Topic: /terrain_cost_grid
- Message: nav_msgs/msg/OccupancyGrid
- Frame: odom (current simulation)
- QoS: RELIABLE, TRANSIENT_LOCAL, depth 1
- Publication: full grid every 2 seconds
- Timestamps: ROS simulation time

## Current Grid Geometry
- Resolution: 0.2 meters/cell
- Width: 150 cells
- Height: 150 cells
- Coverage: 30 x 30 meters
- Origin: (-10.0, -15.0)
- Orientation: identity quaternion
- Data order: row-major, index = y * width + x

## Terrain Values
- -1: unknown or unclassified terrain (reserved)
- 0: ordinary traversable terrain
- 1-99: progressively higher traversal difficulty
- 100: lethal / nontraversable threshold

The synthetic demonstration uses cost 80 for rough
terrain. Nav2 maps this to approximately cost 203.

Unknown terrain must not be treated as confirmed safe.
The current synthetic publisher initializes all cells
to 0 and does not yet publish unknown cells.

## Current Test Region
- X: 9.0 to 11.0 m
- Y: -2.0 to 2.0 m
- Terrain value: 80

The publisher can toggle this region using:
ros2 param set /a1_terrain_grid terrain_enabled true
ros2 param set /a1_terrain_grid terrain_enabled false

## Navigation Integration
Global costmap layers:
1. terrain_layer (StaticLayer)
2. obstacle_layer (LaserScan)
3. inflation_layer

Configuration:
config/nav2_terrain_params.yaml

Combined launch:
ros2 launch rover_navigation terrain_sim.launch.py

The rover description/Gazebo simulation must already
be running before launching Nav2.

## Verified Test Results
Start: (7.0, 0.0)
Goal:  (13.0, 0.0)

Flat terrain:
- Path length: 6.000 m
- Rough-region path poses: 80

Rough terrain:
- Path length: 7.708 m
- Rough-region path poses: 1
- Maximum absolute Y deviation: 2.046 m

## Integration Limitations
- Current publisher generates synthetic terrain only.
- No real terrain classifier is connected yet.
- Frame and grid semantics must be agreed with the
  terrain-processing teammate before physical deployment.
- Actual robot clearance and localization must be
  validated separately.

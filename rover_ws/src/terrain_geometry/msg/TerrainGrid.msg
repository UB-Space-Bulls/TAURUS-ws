# Version 1: fixed, axis-aligned 2.5D grid. Arrays have width*height entries.
# Index = row_y * info.width + column_x (x increases first).
# Header stamp is the INPUT cloud acquisition stamp; frame must be level.
std_msgs/Header header
nav_msgs/MapMetaData info
uint8 GROUND_VALID=1
uint8 OBSTACLE_PRESENT=2
# Bitmask: 0 unknown, 1 ground, 2 obstacle with unknown ground, 3 both.
uint8[] status
# Metres at cell center in header.frame_id. NaN unless GROUND_VALID.
float32[] ground_height
# Radians [0, pi/2), inclination relative to horizontal. NaN if unknown.
float32[] slope
# Metres, RMS perpendicular point-to-local-plane residual. NaN if unknown.
float32[] roughness
uint32[] ground_count
uint32[] obstacle_count

# SLAM localization status from slam_monitor

# Status constants
uint8 UNKNOWN = 0
uint8 LOCALIZING = 1
uint8 LOCALIZED = 2
uint8 LOST = 3
uint8 MAPPING = 4

# Current status
uint8 status

# Human-readable status name
string status_name

# True if map database exists
bool map_exists

# True if /map topic is being published
bool map_available

# Number of consecutive loop closures detected
uint32 loop_closure_count

# Localization confidence (0.0 to 1.0)
float32 confidence

# Time spent attempting localization
float32 localization_duration_sec

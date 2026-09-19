# Current state of the autonomy system

# State constants
uint8 WAITING_FOR_STARTUP = 0
uint8 CHECKING_MAP = 1
uint8 LOOK_AROUND = 2
uint8 MAPPING_MODE = 3
uint8 LOCALIZATION_MODE = 4
uint8 WAITING_FOR_MISSION = 5
uint8 EXECUTING_MISSION = 6
uint8 EXPLORING = 7
uint8 EXPLORATION_COMPLETE = 8
uint8 ERROR = 9

# Current state
uint8 state

# Human-readable state name
string state_name

# SLAM mode: "mapping" (sonar map, rebuilt each boot) or "localization"
string slam_mode

# True if currently executing a mission
bool mission_active

# Current mission ID (empty if none)
string mission_id

# Exploration progress (0.0 to 1.0)
float32 exploration_progress

# Time remaining before auto-exploration starts (0 if not waiting)
float32 mission_timeout_remaining

# Error message (if state == ERROR)
string error_message

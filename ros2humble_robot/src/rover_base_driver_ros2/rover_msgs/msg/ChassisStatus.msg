# This message contains the status of the battery

std_msgs/Header header

# System status constants
uint8 SYSTEM_STATE_ESTOP = 0
uint8 SYSTEM_STATE_FAULT = 1
uint8 SYSTEM_STATE_STANDBY = 2
uint8 SYSTEM_STATE_REMOTE_CONTROL = 3
uint8 SYSTEM_STATE_COMMAND_CONTROL = 4

uint8 system_status
uint8 anti_collision_status
uint8 anti_drop_status
float32 temperature
float32 humidity
uint16 barometer
bool charging_pile
uint8 reserve
uint8 UNIT_ROT_RAD = 0
uint8 UNIT_ROT_DPS = 1

uint8 UNIT_ACC_GS = 0
uint8 UNIT_ACC_MS2 = 4

vehicle_interfaces/Header header
uint8 unit_type 0
float32[4] orientation
float32[3] angular_velocity
float32[3] linear_acceleration
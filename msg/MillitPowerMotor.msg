uint8 MOTOR_MODE_TORQ = 0
uint8 MOTOR_MODE_SPEED = 1
uint8 MOTOR_MODE_POWER = 2

vehicle_interfaces/Header header
uint8 motor_mode 0
int16 rpm 0 # -10000 to 10000
float32 torque 0 #N-m
float32 percentage 0 # 0% to 100%
float32 voltage 0
float32 current 0
float32 temperature 0
bool parking 0

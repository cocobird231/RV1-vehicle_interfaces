uint8 UNIT_PWM = 0
uint8 UNIT_RPM = 1

Header header
uint8 unit_type 0

# The order of following array related to the motor ID of drive, steering and brake motor respectively.
# If the number of elements of each array does not fit the ChassisInfo of the control server, the message will be ignored. 
float64[] drive_motor
float64[] steering_motor
float64[] brake_motor

# The number of elements and orders of parking_signal should be equal to drive_motor. Set element to true to enable the parking signal.
bool[] parking_signal

# Controller frame id.
uint64 controller_frame_id

# Controller interruption.
bool controller_interrupt 0

# Controller name.
string controller_name
# Name discription: <X>D<Y>S<Z>B.
## <X>: number of driving motor.
## <Y>: number of steering motor.
## <Z>: number of braking motor.
uint8 VEHICLE_TYPE_1D1S1B = 1
uint8 VEHICLE_TYPE_4D4S4B = 4
uint8 VEHICLE_TYPE_8D8S8B = 8

uint8 vehicle_type 4

# Chassis architecture.
## The points regard the front-first-left wheel centroid as the original point.
## The wheel ordered from left to right, from front to end. For instance, the order of 4 wheels on the 2D-axis will be like character "Z".
## The distance unit: mm.
Point2d[] wheel_position
Point2d chassis_centroid
Point2d chassis_cg

# Chassis motor mapping data.
## The size of each vector must equal to the number of wheels.
MappingData[] drive_motor_mapping_vec
MappingData[] steering_motor_mapping_vec
MappingData[] brake_motor_mapping_vec

# Chassis motor direction correction.
float64[] drive_motor_correction_vec
float64[] steering_motor_correction_vec
float64[] brake_motor_correction_vec

# Chassis motor value limitation.
MotorValueRange drive_motor_pwm_value
MotorValueRange drive_motor_rpm_value

MotorValueRange steering_motor_pwm_value
MotorValueRange steering_motor_angle_value

MotorValueRange brake_motor_pwm_value
MotorValueRange brake_motor_psi_value
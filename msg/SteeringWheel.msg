uint8 GEAR_PARK = 0
uint8 GEAR_REVERSE = 1
uint8 GEAR_NEUTRAL = 2
uint8 GEAR_DRIVE = 3

Header header

# set to GEAR_XXX.
uint8 gear 0

# -32768 to 32767 represents left-end to right-end respectively.
int16 steering 0

# -32768 to 32767 represents pressed-end to non-pressed respectively.
int16 pedal_throttle 0
int16 pedal_brake 0
int16 pedal_clutch 0

# Preserved functions.
uint8 func_0 0
uint8 func_1 0
int8 func_2 0
int8 func_3 0

# Controller name.
string controller_name
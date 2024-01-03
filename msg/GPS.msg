uint8 UNIT_DMM = 0
uint8 UNIT_DD = 1

uint8 GPS_NONE = 0 # Not positioned
uint8 GPS_SPP = 1 # Single point positioning
uint8 GPS_DGPS = 2 # Differential GPS fixed solution
uint8 GPS_FIXED = 4 # Fixed solution
uint8 GPS_FLOAT = 5 # Floating point solution

vehicle_interfaces/Header header
uint8 unit_type 0
uint8 gps_status 0
float64 latitude 0
float64 longitude 0
float64 altitude 0

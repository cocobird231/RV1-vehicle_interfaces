uint8 GPS_NONE = 0 # Not positioned
uint8 GPS_SPP = 1 # Single point positioning
uint8 GPS_DGPS = 2 # Differential GPS fixed solution
uint8 GPS_FIXED = 4 # Fixed solution
uint8 GPS_FLOAT = 5 # Floating point solution

vehicle_interfaces/Header header
uint8 gps_status 0
float32 latitude 0
float32 longitude 0
float32 altitude 0

uint8 FORMAT_RAW = 0
uint8 FORMAT_JPEG = 1
uint8 FORMAT_PNG = 2

uint8 UNIT_DEPTH_MICRO = 0
uint8 UNIT_DEPTH_MILLI = 1
uint8 UNIT_DEPTH_CENTI = 2
uint8 UNIT_DEPTH_METER = 3

vehicle_interfaces/Header header
uint8 format_type
int16 cvmat_type
uint8 depth_unit_type
uint16 width
uint16 height
uint8[] data
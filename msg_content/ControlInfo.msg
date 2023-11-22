# Control Registration
uint8 MSG_TYPE_STEERING_WHEEL = 0
uint8 MSG_TYPE_CHASSIS = 1

## Add service_name to control list
string service_name

## Set to MSG_TYPE_STEERING_WHEEL or MSG_TYPE_CHASSIS
uint8 msg_type

## Reference timeout in ms
float32 timeout_ms
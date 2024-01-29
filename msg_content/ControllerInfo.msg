# The message type that controller will be sent.
uint8 MSG_TYPE_STEERING_WHEEL = 0
uint8 MSG_TYPE_CHASSIS = 1

# Controller mode. 
## If client mode, the control message should be sent periodically.
## If server mode, the control message will be requested by control server.
uint8 SERVICE_MODE_CLIENT = 0
uint8 SERVICE_MODE_SERVER = 1

# Set to MSG_TYPE_STEERING_WHEEL or MSG_TYPE_CHASSIS
uint8 msg_type 0

# Set to SERVICE_MODE_CLIENT or SERVICE_MODE_SERVER
uint8 service_mode 0

# Add service_name to control list
string service_name

# Reference timeout in ms
float32 timeout_ms

# Reference send/recv period in ms. period_ms should greater than timeout_ms.
float32 period_ms
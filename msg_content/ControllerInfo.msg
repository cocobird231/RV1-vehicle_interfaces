# The message type that controller will be sent.
uint8 MSG_TYPE_STEERING_WHEEL = 0
uint8 MSG_TYPE_CHASSIS = 1

# Controller mode. 
## If topic mode, the control message should be sent from controller.
## If service mode, the control message should be requested by control server.
uint8 CONTROLLER_MODE_TOPIC = 0
uint8 CONTROLLER_MODE_SERVICE = 1

# Publish setting.
uint8 PUB_TYPE_NONE = 0
uint8 PUB_TYPE_CONTROLLER = 1
uint8 PUB_TYPE_CONTROLSERVER = 2
uint8 PUB_TYPE_BOTH = 3

# Set to MSG_TYPE_STEERING_WHEEL or MSG_TYPE_CHASSIS
uint8 msg_type 0

# Set to CONTROLLER_MODE_TOPIC or CONTROLLER_MODE_SERVICE
uint8 controller_mode 0

# Add service_name to control list
string service_name

# Reference timeout in ms
float32 timeout_ms

# Reference send/recv period in ms. period_ms should greater than timeout_ms.
float32 period_ms

# Privileges. Range from 0 to 100.
int8 privilege 100

# Set to PUB_TYPE_XXX
uint8 pub_type 0
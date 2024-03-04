# Topic Device Information

## Topic device type.
### UNKNOWN: Should not be used.
### PUBLISHER: The device is a publisher.
### SUBSCRIPTION: The device is a subscriber.
uint8 DEVICE_TYPE_UNKNOWN = 0
uint8 DEVICE_TYPE_PUBLISHER = 1
uint8 DEVICE_TYPE_SUBSCRIPTION = 2

# ======== Variables Field ========

## Node name
string node_name

## Topic name.
string topic_name

## Topic device type. Set to DEVICE_TYPE_XXX.
uint8 device_type 0
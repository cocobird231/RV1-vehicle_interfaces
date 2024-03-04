# Multi-Interactive node information. This message can be used to control the behavior of node, or describes the status of node.

## Target alive status
### NONE: No action for target alive signal.
### ENABLE: Set target alive signal to enable.
### DISABLE: Set target alive signal to disable.
uint8 TARGET_ALIVE_NONE = 0
uint8 TARGET_ALIVE_ENABLE = 1
uint8 TARGET_ALIVE_DISABLE = 2

## Target activity status
### NONE: No action for target activity signal.
### ENABLE: Set target activity signal to enable.
### DISABLE: Set target activity signal to disable.
uint8 TARGET_ACTIVITY_NONE = 0
uint8 TARGET_ACTIVITY_ENABLE = 1
uint8 TARGET_ACTIVITY_DISABLE = 2

# ======== Variables Field ========

# The interactive node name
string node_name

# Set the target alive state of interactive node to TARGET_ALIVE_XXX.
# For instance, create or close publisher.
uint8 target_alive

# Set the target activity state of interactive node to TARGET_ACTIVITY_XXX.
# For instance, start or stop publisher.
uint8 target_activity

# ======== Read Only Field ========

# For read only. The flag describes whether the interaction of node is available.
bool no_interaction 0

# For read only. The flag describes whether the white list is available.
bool use_white_list 0

# For read only. The list of which server request will be accepted.
# The member of white list should be the service name.
InteractiveNodeMasterPrivilege[] white_list

## ======== Multi-Interactive Node ========

# For read only. The flag describes whether the interactive node supports multiple master.
bool is_multi 0

# For read only. Return all target alive and activity state related to each master.
# Ignored if is_multi set to false.
string[] target_alive_master_vec
uint8[] target_alive_vec
string[] target_activity_master_vec
uint8[] target_activity_vec
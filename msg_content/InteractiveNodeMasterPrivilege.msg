# Interactive node master privilege

# Target alive privilege.
uint8 TARGET_ALIVE_NONE = 0
uint8 TARGET_ALIVE_ENABLE_ONLY = 1
uint8 TARGET_ALIVE_DISABLE_ONLY = 2
uint8 TARGET_ALIVE_ALL = 3

# Target alive privilege.
uint8 TARGET_ACTIVITY_NONE = 0
uint8 TARGET_ACTIVITY_ENABLE_ONLY = 1
uint8 TARGET_ACTIVITY_DISABLE_ONLY = 2
uint8 TARGET_ACTIVITY_ALL = 3

# ======== Variables Field ========

string master_id
uint8 target_alive_privilege 0
uint8 target_activity_privilege 0
string[] node_command
bool request_interactive_node_privilege 0
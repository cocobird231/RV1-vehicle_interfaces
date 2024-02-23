# Control signal for Control Server.

######## Definitions ########

# Controller behavior.

## The action of controller behavior.
### NONE: No action for controller.
### SELECT: Select the controller as main controller.
### REMOVE: Remove the controller from list.
uint8 CONTROLLER_ACTION_NONE = 0
uint8 CONTROLLER_ACTION_SELECT = 1
uint8 CONTROLLER_ACTION_REMOVE = 2

# Control server behavior.

## The action of Control Server.
### NONE: No action for Control Server.
### SET_TIMER: Set Control Server timer status, including output, safety check, idclient check and publish timers.
### SET_PERIOD: Set Control Server timer periods, including output, safety check, idclient check and publish timers.
uint8 SERVER_ACTION_NONE = 0
uint8 SERVER_ACTION_SET_TIMER = 1
uint8 SERVER_ACTION_SET_PERIOD = 2

## Timer status.
### NONE: Ignore timer status change.
### STOP: Stop timer.
### START: Start timer.
uint8 TIMER_STATUS_NONE = 0
uint8 TIMER_STATUS_STOP = 1
uint8 TIMER_STATUS_START = 2

## Timer period.
### If value <= 0, the set period action will be ignored at specific timer.


######## Variables ########

# Controller behavior.

## Set to CONTROLLER_ACTION_XXX.
uint8 controller_action 0

## The the service name of controller which to be selected as main controller or remove from list.
string controller_service_name

# Control Server behavior.

## Set to SERVER_ACTION_XXX.
### The Control Server will ignored set timer status and period actions if set to NONE.
### If multiple actions need to be configured, use 'or' operator to combine the SERVER_ACTION_XXX.
uint8 server_action 0


## The status of output timer. Set to TIMER_STATUS_XXX.
uint8 server_output_timer_status 0

## The period of output signals in _ms. Server will ignore this if value <= 0.
float64 server_output_period_ms 0


## The status of safety check timer. Set to TIMER_STATUS_XXX.
uint8 server_safety_timer_status 0

## The period of safety check in _ms. Server will ignore this if value <= 0.
float64 server_safety_period_ms 0


## The status of idclient check timer. Set to TIMER_STATUS_XXX.
uint8 server_idclient_timer_status 0

## The period of idclient check in _ms. Server will ignore this if value <= 0.
float64 server_idclient_period_ms 0


## The status of publish timer. Set to TIMER_STATUS_XXX.
uint8 server_publish_timer_status 0

## The period of publish in _ms. Server will ignore this if value <= 0.
float64 server_publish_period_ms 0
# Control signal for data server.

######## Definitions ########


# Data server behavior.

## The action of data server.
### NONE: No action for data server.
### SET_TIMER: Set data server timer status, including scan, sampling, dump and countdown timers.
### SET_PERIOD: Set data server timer periods, including scan, sampling, dump and countdown timers.
### STRAT: Start data server record. This function will automatically start the timers with valid period.
### STOP: Stop data server record. This function will automatically start all the timers.
### Priority: STOP -> STRAT -> SET_TIMER -> SET_PERIOD -> NONE. The server will only execute one action per request.
uint8 SERVER_ACTION_NONE = 0
uint8 SERVER_ACTION_STOP = 1
uint8 SERVER_ACTION_STRAT = 2
uint8 SERVER_ACTION_SET_TIMER = 4
uint8 SERVER_ACTION_SET_PERIOD = 8

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


# Data server behavior.

## Set to SERVER_ACTION_XXX.
### The data server will ignored action if set to NONE or invalid value.
uint8 server_action 0


## The status of scan timer. Set to TIMER_STATUS_XXX.
uint8 server_scan_timer_status 0

## The period of scanning in _ms. Server will ignore this if value <= 0.
float64 server_scan_period_ms 0


## The status of data sampling timer. Set to TIMER_STATUS_XXX.
uint8 server_sample_timer_status 0

## The period of sampling in _ms. Server will ignore this if value <= 0.
float64 server_sample_period_ms 0


## The status of dump timer. Set to TIMER_STATUS_XXX.
uint8 server_dump_timer_status 0

## The period of dumping in _ms. Server will ignore this if value <= 0.
float64 server_dump_period_ms 0


## The status of countdown timer. Set to TIMER_STATUS_XXX.
uint8 server_countdown_timer_status 0

## The period of countdown in _ms. Server will ignore this if value <= 0.
float64 server_countdown_period_ms 0
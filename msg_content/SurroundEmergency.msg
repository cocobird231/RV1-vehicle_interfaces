# emergency_percentages from 0.0 to 1.0 represents emergency level from low to high.
# The indices of array from 0 to 7 represents the 8-direction [F, B, L, R, FL, FR, BL, BR] respectively.
# F: forward, B: backward, L: left, R: right, FL: forward-left, FR: forward-right, BL: backward-left, BR: backward-right
# Set value < 0.0 if emergency score not used. The value is considered as 1.0 if value > 1.0
float32[8] emergency_percentages [-1, -1, -1, -1, -1, -1, -1, -1]

# lifetime_ms declares the lifetime of emergency_percentages.
float32 lifetime_ms 500.0
vehicle_interfaces/Header header

# Subscribed topic
string rgb_topic_name
uint64 rgb_frame_id
string depth_topic_name
uint64 depth_frame_id

# Image size correspond to rgb image and depth image
uint16 width
uint16 height

# RoIs and ground line
vehicle_interfaces/Bbox2d[] roi
int16[] ground_line # Pixel position (ignore value < 0). Array size must fit image width

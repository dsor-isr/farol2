# Farol Full Path Message

# Header Information
std_msgs/Header header
# Reference point
geometry_msgs/Point ref_point
# Type of path
int8[] type
# Initial Point
geometry_msgs/Point[] start_point
# Center Point
geometry_msgs/Point[] center_point
# End Point
geometry_msgs/Point[] end_point
# Direction of the path
int8[] adirection
# Desired velocity
float32 velocity
# Gamma start
float32[] gamma_s
# Gamma end
float32[] gamma_e

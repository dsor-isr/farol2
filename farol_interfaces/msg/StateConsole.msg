# State message for console

# Header Information
std_msgs/Header header

# GPS Quality
uint8 gps_good # 0 = Auto; 1 = DGPS; 2 = RTK Float; 3 = RTK Fixed; 255 = no GPS
# IMU Quality
uint8 imu_good # 0 = Bad; 1 = Good/Not Filtered; 2= ; 3 = Very good

# Depth Measurement
float64 depth

# East (console is in ENU, so x is east)
float64 x
# North (console is in ENU, so y is north)
float64 y
# Down
float64 z
# Velocity in X
float64 vx
# Velocity in Y
float64 vy
# Velocity in Z
float64 vz   # Not using
# Surge
float64 surge
# Yaw
float64 yaw
# Pitch
float64 pitch
# Roll
float64 roll
# Yaw Rate
float64 yaw_rate
# Pitch Rate
float64 pitch_rate
# Roll Rate
float64 roll_rate
# Inside Pressure
float64 in_pressure
# Inside Pressure rate
float64 in_pressure_dot
# Battery level
uint8 battery_level
# Altitude
float64 altitude

# Status of the vehicle
uint8 status
uint8 STATUS_FAULT = 0
uint8 STATUS_HPOSITION_OK = 1
uint8 STATUS_VPOSITION_OK = 2
uint8 STATUS_ORIENTATION_OK = 4
uint8 STATUS_VELOCITY_OK = 8
uint8 STATUS_ALL_OK = 15

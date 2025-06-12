# Measurement from sensor to feed back to the navigation filter
std_msgs/Header header

# Measurement type
uint8 MEAS_ORIENTATION = 0
uint8 MEAS_ORIENTATION_RATE = 1
uint8 MEAS_UTM_POSITION = 2
uint8 MEAS_DEPTH = 3
uint8 MEAS_ALTIMETER = 4
uint8 MEAS_ALTITUDE_WGS84 = 5
uint8 MEAS_BODY_VELOCITY_INERTIAL = 6
uint8 MEAS_BODY_VELOCITY_FLUID = 7

uint8 type

# Measurement reading
float64[] value

# Measurement variances
float64[] noise

# Measurement additional data
string data
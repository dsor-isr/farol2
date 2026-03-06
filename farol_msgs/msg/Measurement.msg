# Measurement from sensor to feed back to the navigation filter
std_msgs/Header header

# Measurement type
uint8 MEAS_ATTITUDE = 0
uint8 MEAS_ANGULAR_VELOCITY = 1
uint8 MEAS_ANGULAR_ACCELERATION = 2
uint8 MEAS_UTM_POSITION = 3
uint8 MEAS_GLOBAL_POSITION = 4
uint8 MEAS_DEPTH = 5
uint8 MEAS_ALTIMETER = 11 # FIXME 
uint8 MEAS_ALTITUDE_WGS84 = 6
uint8 MEAS_INERTIAL_VELOCITY = 7    
uint8 MEAS_FLUID_VELOCITY = 9
uint8 MEAS_INERTIAL_ACCELERATION = 10

uint8 type

# Measurement reading
float64[] value

# Measurement variances
float64[] noise

# Measurement additional data
string data 
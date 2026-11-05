std_msgs/Header header

# Position
GCS global_position # Geographic Coordinate System: Latitude, Longitude (degrees)
UTM utm_position # Universal Transverse Mercator: Northing, Easting (m), UTM zone

# Depth
float32 depth

# Altimeter
float32 altimeter 

# Altitude
float32 altitude_wgs84 # ellipsoidal altitude WGS84
LocalDatum altitude_local_datum # local datum altitude

# Velocity over ground
geometry_msgs/Vector3 velocity_over_ground # velocity relative to static frame (NED), expressed in the body (m/s)
float64 speed_over_ground # speed over ground (m/s)
float32 course_over_ground # course over ground (deg)

# Velocity through water
geometry_msgs/Vector3 velocity_through_water # velocity relative to the fluid frame (NED-oriented but moving with water), expressed in the body (m/s)
float64 speed_through_water # speed through water (m/s)

# Attitude
Attitude attitude # [roll pitch yaw] (deg)

# Angular Velocity
geometry_msgs/Vector3 angular_velocity # [p q r] (deg/s)

# Velocity of the current
geometry_msgs/Vector3 current_velocity_ned # estimated current velocity in inertial NED frame (m/s)
float64 current_speed # estimated current speed (m/s)
float64 current_direction # estimated current direction (degrees, 0 = north, positive clockwise)

# Actuator states
float64[] thruster_rpm
float64[] control_surface_angle

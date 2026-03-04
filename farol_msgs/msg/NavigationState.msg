std_msgs/Header header

# Position
GCS global_position # Geographic Coordinate System: Latitude, Longitude (degrees)
UTM utm_position # Universal Transverse Mercator: Northing, Easting (m), UTM zone

# Depth
float32 depth

# Altitude
float32 altimeter # output altimeter
float32 altitude_ellipsoidal # ellipsoidal altitude WGS84
LocalDatum local_datum # local datum altitude

# Velocity
geometry_msgs/Vector3 body_velocity_inertial # velocity relative to static frame (NED), expressed in the body (m/s)
geometry_msgs/Vector3 body_velocity_fluid # velocity relative to the fluid frame (NED-oriented but moving with water), expressed in the body (m/s)
geometry_msgs/Vector3 ned_velocity_inertial # velocity relative to static frame, expressed in the NED frame (m/s)
geometry_msgs/Vector3 ned_velocity_fluid # velocity relative to the fluid expressed in the NED frame (m/s)

# Orientation
geometry_msgs/Vector3 orientation # [roll pitch yaw] (rad)

# Orientation Rate
geometry_msgs/Vector3 orientation_rate # [p q r] (rad/s)

float32 course_angle


# If needed: add variances to position, orientation and other estimated variables
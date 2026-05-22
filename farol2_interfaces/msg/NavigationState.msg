std_msgs/Header header

# Position
GCS global_position # Geographic Coordinate System: Latitude, Longitude (degrees)
UTM utm_position # Universal Transverse Mercator: Northing, Easting (m), UTM zone

# Depth
float32 depth

# Altimeter
float32 altimeter 

# Altitude
float32 altitude_ellipsoidal # ellipsoidal altitude WGS84
LocalDatum local_datum # local datum altitude

# Body Velocities
geometry_msgs/Vector3 velocity_over_ground_body # velocity relative to static frame (NED), expressed in the body (m/s)
geometry_msgs/Vector3 velocity_through_water_body # velocity relative to the fluid frame (NED-oriented but moving with water), expressed in the body (m/s)

# Inertial Velocities
geometry_msgs/Vector3 velocity_over_ground_ned # velocity relative to static frame, expressed in the NED frame (m/s)
geometry_msgs/Vector3 velocity_through_water_ned # velocity relative to the fluid expressed in the NED frame (m/s)
float32 course_over_ground # course over ground (deg)

# Velocity of the current
geometry_msgs/Vector3 current_velocity_inertial # estimated current velocity in inertial NED frame (m/s)

# Attitude
geometry_msgs/Vector3 attitude # [roll pitch yaw] (deg)

# Angular Velocity
geometry_msgs/Vector3 angular_velocity # [p q r] (deg/s)

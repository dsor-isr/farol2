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
geometry_msgs/Vector3 body_velocity_inertial # inertial velocity expressed in the body [u, v, w] (m/s)
geometry_msgs/Vector3 body_velocity_fluid # velocity relative to the fluid expressed in the body (m/s)

# Orientation
geometry_msgs/Vector3 orientation # [roll pitch yaw] (rad)

# Orientation Rate
geometry_msgs/Vector3 orientation_rate # [p q r] (rad/s)

# If needed: add variances to position, orientation and other estimated variables
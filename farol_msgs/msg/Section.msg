# Farol Path Section Type Message 

# Hader information
std_msgs/Header header
# All other points are relative to this
float64 xrefpoint     # all other points are relative to this
float64 yrefpoint

# Starting point
float64 xs            # start
float64 ys

# Center point
float64 xc            # center
float64 yc

# End point
float64 xe            # end
float64 ye

# Desired velocity
float64 vl            

# Direction of the arc
float64 direction     
# Radius of the arc
float64 r0

# Gamma start
float64 gamma_s
# Gamma end
float64 gamma_e

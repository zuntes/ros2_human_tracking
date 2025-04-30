# Message defining a detected person

# ID of the detection
uint64  detection_id

# Confident of the detection
float64 confidence

# 3D pose (position + orientation) of the *center* of the detection
geometry_msgs/PoseWithCovariance    pose    



# Message defining a tracked person

# ID of a track
uint64      track_id        # unique identifier of the target, consistent over time

# If the track is currently not observable (lost detection, occluded, ...)
bool        is_tracked     

# If the track is matched by a detection
bool        is_matched      

# ID of corresponding detection in the current cycle (undefined if the detection lost)
uint64      detection_id   

# Age of the track
builtin_interfaces/Duration    age            

# The following fields are extracted from the Kalman state x and its covariance C

# Pose of the track 
geometry_msgs/PoseWithCovariance    pose   

# Velocity of the track 
geometry_msgs/TwistWithCovariance   twist     
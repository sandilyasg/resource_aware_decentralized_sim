#Poses (x, y, theta): float64[] poses
#Pose covariances (2x2 matrix flattened to 1D array): float64[] pose_covariances 
#Point locations (x, y): float64[] points  
#Point covariances (2x2 matrix flattened to 1D array): float64[] point_covariances
#Index arrays to align with above: int32[] keys_poses, int32[] keys_points
#gtsam_msgs/OptimizationOutput.msg

Header header
float64[] poses
float64[] pose_covariances
float64[] points
float64[] point_covariances
int32[] keys_poses
int32[] keys_points
int32[] keys_observed_points
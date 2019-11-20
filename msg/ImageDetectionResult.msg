# Same header as the image
std_msgs/Header header
# The ID of the detected tags.
uint64[] ids
# The pose of each tag
geometry_msgs/Transform[] poses
# The residual error of the pose estimation of each tag
float64[] residuals

# 点云团外围框
# contains a bounding box, which is essentially a box somewhere in space
# used here ususally for the outlier-invariant bounding box of a cluster of points

#the pose of the box frame
geometry_msgs/PoseStamped pose_stamped

#the dimensions of the box
geometry_msgs/Vector3 dimensions

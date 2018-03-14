#放置位置消息
# A name for this grasp
string id

# The internal posture of the hand for the grasp
# positions and efforts are used
sensor_msgs/JointState post_place_posture

# The position of the end-effector for the grasp relative to a reference frame 
# (that is always specified elsewhere, not in this message)
geometry_msgs/PoseStamped place_pose

# The approach motion
GripperTranslation approach

# The retreat motion
GripperTranslation retreat

# an optional list of obstacles that we have semantic information about
# and that can be touched/pushed/moved in the course of grasping
string[] allowed_touch_objects

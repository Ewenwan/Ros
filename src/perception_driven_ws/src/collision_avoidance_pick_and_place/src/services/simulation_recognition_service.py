#-*- coding:utf-8 -*-
#!/usr/bin/env python
# 仿真识别服务
import roslib; roslib.load_manifest('collision_avoidance_pick_and_place')
import rospy
import tf
import shape_msgs
import collision_avoidance_pick_and_place.srv
from collision_avoidance_pick_and_place.srv import *


# 目标识别服务 constants
TARGET_RECOGNITION_SERVICE = "target_recognition";

#variables
tf_listener = None

# 服务回调函数 server callback
def recognition_callback(req):
	
	ar_frame = req.ar_tag_frame_id
	world_frame = req.world_frame_id
	shape = req.shape


	# response object
	res = GetTargetPoseResponse()
	
	# lookup tranform
	try:
		(trans,rot) = tf_listener.lookupTransform(world_frame,ar_frame,rospy.Time(0))
		rospy.loginfo("ar_frame '%s' relative to '%s' detected",ar_frame,world_frame)		
		
		#modifying height
		height = shape.dimensions[2] if (len(shape.dimensions)==3) else trans[2]

		# creating pose
		pose = geometry_msgs.msg.Pose()
		pose.position.x = trans[0]
		pose.position.y = trans[1]
		pose.position.z = height
		pose.orientation.x = rot[1]
		pose.orientation.y = rot[2]
		pose.orientation.z = rot[3]
		pose.orientation.w = rot[0]


		res.target_pose = pose
		res.succeeded = True	
		

	except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
		rospy.logerr("ar_frame '%s' relative to '%s' not found",ar_frame,world_frame)
		res.succeeded = False	
		
	
	return res

if __name__ == "__main__":


	rospy.init_node("simulation_recognition_node")

	# creating listener
	tf_listener = tf.TransformListener()

	# creating server
	server = rospy.Service(TARGET_RECOGNITION_SERVICE,GetTargetPose,recognition_callback)
	
	rospy.loginfo("recognition service server ready")
	while not rospy.is_shutdown():
	
		rospy.sleep(0.2)
		






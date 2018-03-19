#-*- coding:utf-8 -*-
#!/usr/bin/env python
# 障碍物数据发布
import argparse
import sys
import shlex
import roslib; roslib.load_manifest('collision_avoidance_pick_and_place')
import rospy
import tf
import shape_msgs
import moveit_msgs
import std_msgs.msg
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose


#constants
COLLISION_OBJECT_TOPIC = "collision_object";
PACKAGE_PATH = roslib.packages.get_pkg_dir('collision_avoidance_pick_and_place')

def create_collision_object(shape_type,pos,size,frame_id,op,object_id):

	col = CollisionObject()
	col.id = object_id
	col.operation = op
	col.header.frame_id=frame_id

	# create primitive
	primitive = SolidPrimitive()
	primitive.type = shape_type
	primitive.dimensions = size

	# create pose
	pose = Pose()
	pose.position.x = pos[0]
	pose.position.y = pos[1]
	pose.position.z = pos[2]
	pose.orientation.x = pose.orientation.y = pose.orientation.z = 0
	pose.orientation.w = 1


	col.primitives = [primitive]
	col.primitive_poses = [pose]

	return col

def parse_arguments(args):

	print "type: %s , size: [%s], position: [%s], frameid: %s, id: %s" %(args.type,','.join(map(str,args.size)),
	','.join(map(str,args.position)),args.frameid,args.id)
	msg = create_collision_object(args.type,args.position,args.size,args.frameid,args.operation,args.id)
	return msg


if __name__ == "__main__":


	rospy.init_node("collision_object_publisher")

	# create publisher
	collision_publisher = rospy.Publisher(COLLISION_OBJECT_TOPIC,CollisionObject)

	# argument parser
	parser=argparse.ArgumentParser(description="Collision Object description")

	# adding arguments
	parser.add_argument("-t","--type",type=int,help="primitive type: BOX=1, SPHERE=2, CYLINDER=3, CONE=4",choices=[1,2,3,4])
	parser.add_argument("-s","--size",type=float, metavar='N',nargs=3,help="size: [l w h]")
	parser.add_argument("-p","--position",type=float,metavar='N',nargs=3,help="position: [x y z]")
	parser.add_argument("-f","--frameid",type=str,help="frame id")
	parser.add_argument("-o","--operation",type=int,help="operations: ADD=0, DELETE=1 ",choices=[0,1])
	parser.add_argument("-i","--id",type=str,help="object id")
	parser.add_argument("-y","--file",type=str,help="file with object entries")
	parser.add_argument("-l","--loop",action='store_true',help="loop and accept new entries from terminal")	
	parser.add_argument("-q","--quit",action='store_true',help="quit script")

	args = parser.parse_args()

	# wait for subscribers
	max_attempts = 20
	num_attempts = 0	
	while (collision_publisher.get_num_connections() < 1 ):
		rospy.loginfo("waiting for subscribers to %s topic" % (COLLISION_OBJECT_TOPIC))
		rospy.sleep(2.0)
		num_attempts= num_attempts+1

		if num_attempts > max_attempts or rospy.is_shutdown():
			rospy.logerr("No subscribers for topic %s found" % (COLLISION_OBJECT_TOPIC))
			sys.exit()

	rospy.loginfo("Subscribers for topic %s found" % (COLLISION_OBJECT_TOPIC))

	if args.loop:




		while True:
			arg_string=raw_input('Enter object: ')
			args = parser.parse_args(shlex.split(arg_string))
			if not args.quit:

				if args.file != None:
					fname=PACKAGE_PATH + '/' + args.file
					f = open(fname)
					for line in f:
						args = parser.parse_args(line)
						msg = parse_arguments(args)
						collision_publisher.publish(msg)
				else:
					msg = parse_arguments(args)
					collision_publisher.publish(msg)

			else:
				break

	else:

		if args.file != None:
			fname=PACKAGE_PATH + '/' + args.file
			rospy.loginfo("Opening file %s" %(fname))
			f = open(fname)
			for line in f:
				args = parser.parse_args(line.split(' '))
				msg = parse_arguments(args)
				collision_publisher.publish(msg)
		else:

			if args.frameid != None and args.type != None and args.id != None :

				msg = parse_arguments(args)
				collision_publisher.publish(msg)
			else:
				print "No arguments passed, exiting"



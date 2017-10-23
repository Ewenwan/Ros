#!/usr/bin/env python
#-*- coding:utf-8 -*-
""" move_base_square.py - Version 1.1 2013-12-20
    Command a robot to move in a square using move_base actions..
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.
    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.htmlPoint
      
"""

import rospy      # ros system depends roscpp 系统文件
import actionlib  # 运动库
from actionlib_msgs.msg import * #运动消息
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
# Twist:linear.x linear.y linear.z    Point: x,y,z   Quaternion: x,y,z,w 
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal  # move_base 消息
from tf.transformations import quaternion_from_euler         # 坐标转换 里的 角度转换成四元素
from visualization_msgs.msg import Marker                    # 目标点的可视化 marker 消息库
from math import radians, pi                                 # 数学常量

class MoveBaseSquare():
    # standard class initialization
    def __init__(self):
        # 建立节点 Creat a node named nav_test 
        rospy.init_node('nav_test', anonymous=False)
        # 退出程序 Set rospy to execute a shutdown function when exiting    
        rospy.on_shutdown(self.shutdown)        
        # 导航正方形 尺寸 How big is the square we want the robot to navigate?
        square_size = rospy.get_param("~square_size", 1.0) # meters
        
        # 方位 列表 Create a list to hold the target quaternions (orientations)
        quaternions = list()     # target uaternions (orientations)  
        # z轴欧拉角 First define the corner orientations as Euler angles
        euler_angles = (pi/2, pi, 3*pi/2, 0) # location orientations       
        # 欧拉角转换到四元素 Then convert the angles to quaternions
        for angle in euler_angles:         # x，y轴角度为0
            q_angle = quaternion_from_euler(0, 0, angle, axes='sxyz')
            q = Quaternion(*q_angle)
            quaternions.append(q) # 添加到列表里generate the target quaternions (orientations)
        
        ## 目标点位置列表 Create a list to hold the waypoint poses
        waypoints = list()  # target pose(position +  quaternions (orientations) )     
        # Append each of the four waypoints to the list.  Each waypoint
        # is a pose consisting of a position and orientation in the map frame.
        waypoints.append(Pose(Point(square_size, 0.0, 0.0), quaternions[0]))#(1,0,0)点
        waypoints.append(Pose(Point(square_size, square_size, 0.0), quaternions[1]))#(1,1,0)点
        waypoints.append(Pose(Point(0.0, square_size, 0.0), quaternions[2]))#(0,1,0)点
        waypoints.append(Pose(Point(0.0, 0.0, 0.0), quaternions[3]))#(0,0,0)点
        
        # 目标点位置3D图显示初始化  Initialize the visualization markers for RViz
        self.init_markers()
        
        # 添加可视化目标位置 Set a visualization marker at each waypoint        
        for waypoint in waypoints:           
            p = Point()            # initialize the variable
            p = waypoint.position  # copy the position 
            self.markers.points.append(p)
            
        # 发布命令到手动控制 的话题上 Publisher to manually control the robot (e.g. to stop it, queue_size=5)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)      
        # 订阅话题 Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)      
        rospy.loginfo("Waiting for move_base action server...")       
        # Wait 60 seconds for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(60))       
        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting navigation test")
        
        # Initialize a counter to track waypoints
	## 记录所到的目标点数 
        i = 0        
        # Cycle through the four waypoints
        while i < 4 and not rospy.is_shutdown():
            # Update the marker display
            self.marker_pub.publish(self.markers)
            
            # Intialize the waypoint goal
            goal = MoveBaseGoal()           
            # Use the map frame to define goal poses
            goal.target_pose.header.frame_id = 'map'   ##帧ID        
            # Set the time stamp to "now"
            goal.target_pose.header.stamp = rospy.Time.now()  ##时间戳          
            # Set the goal pose to the i-th waypoint
            goal.target_pose.pose = waypoints[i]       ##目标点位     
            # Start the robot moving toward the goal
            self.move(goal)                            ##移动
            
            i += 1
        
    def move(self, goal):
            # Send the goal pose to the MoveBaseAction server
            self.move_base.send_goal(goal)           
            # Allow 1 minute to get there
            finished_within_time = self.move_base.wait_for_result(rospy.Duration(60)) 
            
            # If we don't get there in time, abort the goal
            if not finished_within_time:
                self.move_base.cancel_goal()
                rospy.loginfo("Timed out achieving goal")
            else:
                # We made it!
                state = self.move_base.get_state()
                if state == GoalStatus.SUCCEEDED:
                    rospy.loginfo("Goal succeeded!")
                    
    def init_markers(self):
        # Set up our waypoint markers
        marker_scale = 0.2
        marker_lifetime = 0 # 0 is forever
        marker_ns = 'waypoints'
        marker_id = 0
        marker_color = {'r': 1.0, 'g': 0.7, 'b': 1.0, 'a': 1.0} #dictionary
        
        # Define a marker publisher.
        self.marker_pub = rospy.Publisher('waypoint_markers', Marker, queue_size=5)
        
        # Initialize the marker points list.
        self.markers = Marker()
        self.markers.ns = marker_ns
        self.markers.id = marker_id
        self.markers.type = Marker.CUBE_LIST##正方形 脚印
        self.markers.action = Marker.ADD
        self.markers.lifetime = rospy.Duration(marker_lifetime)
        self.markers.scale.x = marker_scale
        self.markers.scale.y = marker_scale
        self.markers.color.r = marker_color['r']
        self.markers.color.g = marker_color['g']
        self.markers.color.b = marker_color['b']
        self.markers.color.a = marker_color['a']
        
        self.markers.header.frame_id = 'odom'
        self.markers.header.stamp = rospy.Time.now()
        self.markers.points = list()

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        # Cancel any active goals
        self.move_base.cancel_goal()
        rospy.sleep(2)
        # Stop the robot
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        MoveBaseSquare()
    except rospy.ROSInterruptException:
	rospy.loginfo("Navigation test finished.")


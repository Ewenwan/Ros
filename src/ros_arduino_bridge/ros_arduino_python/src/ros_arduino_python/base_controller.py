#-*- coding:utf-8 -*-
#!/usr/bin/python
"""
    A base controller class for the Arduino microcontroller
    
    Borrowed heavily from Mike Feguson's ArbotiX base_controller.py code.
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2010 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses
"""
import roslib; roslib.load_manifest('ros_arduino_python')
import rospy
import os

from math import sin, cos, pi     #运动模型解析
from geometry_msgs.msg import Quaternion, Twist, Pose # 全局速度和位置
from nav_msgs.msg import Odometry                     # 里程记信息
from tf.broadcaster import TransformBroadcaster       # 坐标广播
 
""" Class to receive Twist commands and publish Odometry data """
class BaseController:
    def __init__(self, arduino, base_frame, name="base_controllers"):
        self.arduino = arduino
        self.name = name
        self.base_frame = base_frame
        self.rate = float(rospy.get_param("~base_controller_rate", 10))#控制器控制频率
        self.timeout = rospy.get_param("~base_controller_timeout", 1.0)#
        self.stopped = False
                 
        pid_params = dict()
        pid_params['wheel_diameter'] = rospy.get_param("~wheel_diameter", "") # 轮子直径
        pid_params['wheel_track'] = rospy.get_param("~wheel_track", "")       # 两轮 间距
        pid_params['encoder_resolution'] = rospy.get_param("~encoder_resolution", "") # 编码器分辨率
        pid_params['gear_reduction'] = rospy.get_param("~gear_reduction", 1.0)        # 减速齿轮
        pid_params['Kp'] = rospy.get_param("~Kp", 20) # 比例
        pid_params['Kd'] = rospy.get_param("~Kd", 12) # 微分
        pid_params['Ki'] = rospy.get_param("~Ki", 0)  # 积分
        pid_params['Ko'] = rospy.get_param("~Ko", 50) #
        
        self.accel_limit = rospy.get_param('~accel_limit', 0.1) # 加速度限制（增量式期望速度 单限值 简单控制算法参数）
        self.motors_reversed = rospy.get_param("~motors_reversed", False)# 轮子反向
        
        # Set up PID parameters and check for missing values
        self.setup_pid(pid_params)
            
        # How many encoder ticks are there per meter?
        self.ticks_per_meter = self.encoder_resolution * self.gear_reduction  / (self.wheel_diameter * pi) # 每米 编码器的读数 变化
        
        # What is the maximum acceleration we will tolerate when changing wheel speeds?
        self.max_accel = self.accel_limit * self.ticks_per_meter / self.rate # 最大加速度   转 / 控制周期
                
        # Track how often we get a bad encoder count (if any)
        self.bad_encoder_count = 0
                        
        now = rospy.Time.now()    
        self.then = now          # 上次时间 time for determining dx/dy
        self.t_delta = rospy.Duration(1.0 / self.rate)# 控制周期
        self.t_next = now + self.t_delta # 下次控制时间开始

        # 编码器读取 上次读书记录     
        self.enc_left = None            # encoder readings
        self.enc_right = None
        # 全局 速度
        self.x = 0                      # position in xy plane
        self.y = 0
        self.th = 0                     # rotation in radians
        # 轮子的速度
        self.v_left = 0                 # 实际调整的速度
        self.v_right = 0
        self.v_des_left = 0             # 期望的 左右轮速度
        self.v_des_right = 0
        self.last_cmd_vel = now         # 命令发布时间

        # 订阅 cmd_vel 得到全局的 线速度 和角速度命令   回调函数（一接收到速度指令后进行解运动模型得到哥哥轮子的速度）
        rospy.Subscriber("cmd_vel", Twist, self.cmdVelCallback)
        
        # 机器人自身的 速度信息 来自编码器（里程记信息） Clear any old odometry info
        self.arduino.reset_encoders()
        
        # 广播里程信息（速度反馈信息）到相应的话题上   Set up the odometry broadcaster
        self.odomPub = rospy.Publisher('odom', Odometry, queue_size=5)
        self.odomBroadcaster = TransformBroadcaster()
        
        rospy.loginfo("Started base controller for a base of " + str(self.wheel_track) + "m wide with " + str(self.encoder_resolution) + " ticks per rev")
        rospy.loginfo("Publishing odometry data at: " + str(self.rate) + " Hz using " + str(self.base_frame) + " as base frame")
        
    def setup_pid(self, pid_params):
        # Check to see if any PID parameters are missing
        missing_params = False
        for param in pid_params:
            if pid_params[param] == "": # 是否有没有设置的pid参数
                print("*** PID Parameter " + param + " is missing. ***")
                missing_params = True   # 丢失pid控制参数
        
        if missing_params:
            os._exit(1)
        # 物理尺寸     
        self.wheel_diameter = pid_params['wheel_diameter'] # 轮子半径
        self.wheel_track = pid_params['wheel_track']       # 两轮间距 
        self.encoder_resolution = pid_params['encoder_resolution']# 编码器分辨率
        self.gear_reduction = pid_params['gear_reduction']        # 减速齿轮相关
        
        # pid控制器参数
        self.Kp = pid_params['Kp']
        self.Kd = pid_params['Kd']
        self.Ki = pid_params['Ki']
        self.Ko = pid_params['Ko']
        
        self.arduino.update_pid(self.Kp, self.Kd, self.Ki, self.Ko)

    # 发布反馈的 机器人自身的里程记录信息，根据发布在cmd_vel话题上的全局速度解得期望轮子速度 ，发布相应 的 轮子的速度控制命令到 Arduino下位机上
    def poll(self):
        now = rospy.Time.now()
        # 是否到更新时间
        if now > self.t_next:  # 到达 控制 时间 周期
            
         #读取 编码器的值 反解运动控制模型  得到 机器人的里程记信息
            try:
                left_enc, right_enc = self.arduino.get_encoder_counts() # 两个轮子的 当前编码器计数
            except:
                self.bad_encoder_count += 1                             # 不正常 编码器计数
                rospy.logerr("Encoder exception count: " + str(self.bad_encoder_count))
                return
                            
            dt = now - self.then # 时间差
            self.then = now      # 更新上次时间
            dt = dt.to_sec()     # 转换到秒
            
         # 计算里程记信息（机器人自身 轮子的速度信息）
            if self.enc_left == None:# 没有读取到编码器信息
                dright = 0
                dleft = 0
            else:
                dright = (right_enc - self.enc_right) / self.ticks_per_meter # 右轮 速度
                dleft = (left_enc - self.enc_left) / self.ticks_per_meter    # 左轮 速度
            self.enc_right = right_enc #更新 编码器计数
            self.enc_left = left_enc           
            # 两轮车 运动模型  反解    三轮车 不一样
            dxy_ave = (dright + dleft) / 2.0 # 平均速度
            dth = (dright - dleft) / self.wheel_track # 速度差（弧长）/（转弯半径，即两轮间距）  转弯角度
            vxy = dxy_ave / dt               # 线速度大小
            vth = dth / dt                   # 角速度大小               
            #  得到位置信息 
            if (dxy_ave != 0): 
                dx = cos(dth) * dxy_ave
                dy = -sin(dth) * dxy_ave
                self.x += (cos(self.th) * dx - sin(self.th) * dy)  #当前位置
                self.y += (sin(self.th) * dx + cos(self.th) * dy)    
            if (dth != 0):
                self.th += dth 
            # 四元素信息
            quaternion = Quaternion()
            quaternion.x = 0.0 
            quaternion.y = 0.0
            quaternion.z = sin(self.th / 2.0)
            quaternion.w = cos(self.th / 2.0)    
            # 发布坐标变换信息
            self.odomBroadcaster.sendTransform(
                (self.x, self.y, 0), 
                (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
                rospy.Time.now(),
                self.base_frame,
                "odom"
                )
            # 发布里程记 信息（位置 四元素 速度（线速度+角速度）
            odom = Odometry()
            odom.header.frame_id = "odom"
            odom.child_frame_id = self.base_frame
            odom.header.stamp = now
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0
            odom.pose.pose.orientation = quaternion
            odom.twist.twist.linear.x = vxy
            odom.twist.twist.linear.y = 0
            odom.twist.twist.angular.z = vth
            self.odomPub.publish(odom)
            
        # 根据期望的速度（cmd_vel上的命令） 向arduino发送控制速度信息（逐步调整命令速度到 下位机单片机，单片机在用pid控制算法是速度精确跟随上位机发布的指令）
            #期望的轮子速度（v_des_left、v_des_right）会在 订阅话题"cmd_vel"节点的回调函数cmdVelCallback里得到更新 
            if now > (self.last_cmd_vel + rospy.Duration(self.timeout)):
                self.v_des_left = 0  #超过读取时间了，没有速度信息发布到话题上，即为 停止状态
                self.v_des_right = 0
                
            # 左轮子速度逐步调整置 期望左轮速度  
            if self.v_left < self.v_des_left:  # 当前速度小于期望速度 则增加
                self.v_left += self.max_accel     # + 增量 
                if self.v_left > self.v_des_left: 
                    self.v_left = self.v_des_left # 最大限制（最大不超过期望速度）                    
            else:                              # 当前速度大于期望速度 则减小
                self.v_left -= self.max_accel     # - 增量
                if self.v_left < self.v_des_left: # 最小限制（最小不低于期望速度）
                    self.v_left = self.v_des_left
            # 右轮子速度逐步调整置 期望右轮速度  
            if self.v_right < self.v_des_right:
                self.v_right += self.max_accel
                if self.v_right > self.v_des_right:
                    self.v_right = self.v_des_right
            else:
                self.v_right -= self.max_accel
                if self.v_right < self.v_des_right:
                    self.v_right = self.v_des_right
            
            # 向下位机发送期望的轮子速度 根据编码器的反馈信息使用 PID 控制跟随期望速度
            if not self.stopped:
                self.arduino.drive(self.v_left, self.v_right)
                
            self.t_next = now + self.t_delta# 下次循环时间
    # 停车       
    def stop(self):
        self.stopped = True
        self.arduino.drive(0, 0)
    
    #  根据 cmd_vel话题上发布的 全局 速度命令 根据 运动学模型 得到每个轮子的 期望速度       
    def cmdVelCallback(self, req):
        # Handle velocity-based movement requests
        self.last_cmd_vel = rospy.Time.now()
        
        # 控制指令  的全局 线速度 和角 速度
        x = req.linear.x         # m/s
        th = req.angular.z       # rad/s

        if x == 0:    # 原地旋转
            # Turn in place
            right = th * self.wheel_track  * self.gear_reduction / 2.0# 得到轮子的转速
            left = -right
        elif th == 0: # 直线行走
            # Pure forward/backward motion
            left = right = x
        else:
            # 旋转 + 径直
            left = x - th * self.wheel_track  * self.gear_reduction / 2.0
            right = x + th * self.wheel_track  * self.gear_reduction / 2.0
            
        self.v_des_left = int(left * self.ticks_per_meter / self.arduino.PID_RATE)
        self.v_des_right = int(right * self.ticks_per_meter / self.arduino.PID_RATE)
        

        

    

    

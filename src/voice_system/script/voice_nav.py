#!/usr/bin/env python
# -*- coding:utf-8 -*-
"""
根据语言识别结果的发布话题上的控制命令，发布速度命令到cmd_vel话题上，地盘接收到速度命令控制电机转动前进
"""

import rospy
from geometry_msgs.msg import Twist # 速度命令消息类型
from std_msgs.msg import String
from math import copysign           # 

# 自定义语音控制的类
class VoiceNav:
    def __init__(self):
        rospy.init_node('voice_nav') # 节点名
        
        rospy.on_shutdown(self.cleanup)
        
        # 一些参数，launch文件或配置文件可修改
        self.max_speed = rospy.get_param("~max_speed", 0.8)                # 最大线速度 m/s
        self.max_angular_speed = rospy.get_param("~max_angular_speed", 1.5)# 最大角速度 
        self.speed = rospy.get_param("~start_speed", 0.3)                  # 前进 开始速度
        self.angular_speed = rospy.get_param("~start_angular_speed", 0.5)  # 旋转 开始角速度
        self.linear_increment = rospy.get_param("~linear_increment", 0.05) # 线速度  每次增加
        self.angular_increment = rospy.get_param("~angular_increment", 0.4)# 角速度  每次增加
        
        # 发布频率
        self.rate = rospy.get_param("~rate", 5)# 指令发布频率
        r = rospy.Rate(self.rate)       
        # 语音控制  标志  默认开启
        self.paused = False      
        # 初始化要发布在cmd_vel话题上的速度类型
        self.cmd_vel = Twist()
        # 发布话题                            话题名    消息类型     队列大小
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)       
        # 订阅语言识别发布话题                     类型     回调函数
        rospy.Subscriber('/recognizer/output', String, self.speech_callback)
        
        # 控制命令 关键字 
        self.keywords_to_command = {'停止': ['停止', '停下来', '停止 前进', '关闭', '睡觉', '停', '禁止', '关闭', 'help me'],
                                    '减速': ['减慢', '慢', '减慢 速度', '变慢', '慢下来', '慢速', '减速'],
                                    '加速': ['加速', '快', '加快 速度', '加快', '加速上去', '加速 前进'],
                                    '前进': ['前进', '向前 移动', '向前 走', '过来', '全速 前进'],
                                    '后退': ['后退', '向后 移动', '向后 走', '倒退','全速 倒退'],
                                    '正转': ['正转', '正向 自转', '正向 旋转', '原地 自转', '转圈', '正向'],
                                    '反转': ['反转', '反向 自转', '反向 旋转', '原地 反转', '反向'],
                                    '左转': ['向左 转弯', '左转', '向左 移动', '向左 走', '左 转弯'],
                                    '右转': ['向右 转弯', '右转', '向右 移动', '向左 走', '左 转弯'],
                                    '当前位置': ['位置', '当前 位置', '报告 位置', '汇报 位置'],
                                    '当前任务': ['任务', '当前 任务', '报告 任务', '回报 任务'],
                                    '当前状态': ['当前状态', '状态'],
                                    '关语控': ['关闭 语音 控制', '禁用 语音 控制'],
                                    '开语控': ['打开 语音 控制', '启用 语音 控制']}
        
        rospy.loginfo("已经准备好接受语言控制命令")
        
        # 按发送频率发送命令
        while not rospy.is_shutdown():
            self.cmd_vel_pub.publish(self.cmd_vel)
            r.sleep()                       
    #  查找命令      
    def get_command(self, data):
        # 在 控制命令 关键字 中查找 有关控制命令
        for (command, keywords) in self.keywords_to_command.iteritems():
            for word in keywords:
                if data.find(word) > -1:
                    return command
                
    # 回调函数  根据语言命令 执行 发送 速度指令 或其他内容    
    def speech_callback(self, msg):
        # 从语音识别话题上 得到指令
        command = self.get_command(msg.data)# 语言识别话题上的消息        
        # 显示 语言识别信息
        rospy.loginfo("Command: " + str(command))
        
        # 语音控制 开关
        if command == '关语控':
            self.paused = True
        elif command == '开语控':
            self.paused = False
        
        if self.paused:
            return       
        
        if command == '前进':    
            self.cmd_vel.linear.x = self.speed
            self.cmd_vel.angular.z = 0
            
        elif command == '反转':
            self.cmd_vel.linear.x = 0
            self.cmd_vel.angular.z = self.angular_speed
                
        elif command == '正转':  
            self.cmd_vel.linear.x = 0      
            self.cmd_vel.angular.z = -self.angular_speed
            
        elif command == '左转':
            if self.cmd_vel.linear.x != 0:
                self.cmd_vel.angular.z += self.angular_increment
            else:        
                self.cmd_vel.angular.z = self.angular_speed
                
        elif command == '右转':    
            if self.cmd_vel.linear.x != 0:
                self.cmd_vel.angular.z -= self.angular_increment
            else:        
                self.cmd_vel.angular.z = -self.angular_speed
                
        elif command == '后退':
            self.cmd_vel.linear.x = -self.speed
            self.cmd_vel.angular.z = 0
            
        elif command == '停止': 
            # Stop the robot!  Publish a Twist message consisting of all zeros.         
            self.cmd_vel = Twist()
        
        elif command == '加速':
            self.speed += self.linear_increment
            self.angular_speed += self.angular_increment
            if self.cmd_vel.linear.x != 0:
                self.cmd_vel.linear.x += copysign(self.linear_increment, self.cmd_vel.linear.x)
            if self.cmd_vel.angular.z != 0:
                self.cmd_vel.angular.z += copysign(self.angular_increment, self.cmd_vel.angular.z)
            
        elif command == '减速':
            self.speed -= self.linear_increment
            self.angular_speed -= self.angular_increment
            if self.cmd_vel.linear.x != 0:
                self.cmd_vel.linear.x -= copysign(self.linear_increment, self.cmd_vel.linear.x)
            if self.cmd_vel.angular.z != 0:
                self.cmd_vel.angular.z -= copysign(self.angular_increment, self.cmd_vel.angular.z)


        else:
            return

        self.cmd_vel.linear.x = min(self.max_speed, max(-self.max_speed, self.cmd_vel.linear.x))
        self.cmd_vel.angular.z = min(self.max_angular_speed, max(-self.max_angular_speed, self.cmd_vel.angular.z))

    def cleanup(self):
        # 停止速度
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        rospy.sleep(1)

if __name__=="__main__":
    try:
        VoiceNav()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("语言导航结束")


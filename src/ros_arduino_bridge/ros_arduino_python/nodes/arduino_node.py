#-*- coding:utf-8 -*-
#!/usr/bin/python
"""
    A ROS Node for the Arduino microcontroller
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
"""

import rospy
from ros_arduino_python.arduino_driver import Arduino # 板子连接 发送读取信息 
from ros_arduino_python.arduino_sensors import *      # 配合Arduino 底层库 各种传感器的 读写函数库
from ros_arduino_msgs.srv import *
from ros_arduino_python.base_controller import BaseController # 全局 速度命令 与 本地 轮子线速度等转换关系
from geometry_msgs.msg import Twist
import os, time
import thread
from serial.serialutil import SerialException # 串口通信

class ArduinoROS():
    def __init__(self):
        # 初始化节点 设置 日志 等级
        rospy.init_node('arduino', log_level=rospy.INFO)

        # 得到实际的节点名称 （参数设置文件可能会更改名字）
        self.name = rospy.get_name()

        # 关闭时清理节点
        rospy.on_shutdown(self.shutdown)

        self.port = rospy.get_param("~port", "/dev/ttyACM0")# 端口
        self.baud = int(rospy.get_param("~baud", 57600))    # 波特率
        self.timeout = rospy.get_param("~timeout", 0.5)     # 等待时间
        self.base_frame = rospy.get_param("~base_frame", 'base_link')# 设置参考坐标系（里程记）

        # 主循环 的频率 要高于 传感器的 检测频率
        self.rate = int(rospy.get_param("~rate", 50))
        r = rospy.Rate(self.rate)

        # 传感器信息发布频率     
        self.sensorstate_rate = int(rospy.get_param("~sensorstate_rate", 10))
        # 是否使用 ros内置的 基础控制器
        self.use_base_controller = rospy.get_param("~use_base_controller", False)
        
        # 设置下一次发布传感器信息的时间
        now = rospy.Time.now()
        self.t_delta_sensors = rospy.Duration(1.0 / self.sensorstate_rate)
        self.t_next_sensors = now + self.t_delta_sensors
        
        # 初始化 姿态信息（线速度、角速度）
        self.cmd_vel = Twist()
  
        # 发布速度指令消息到 cmd_vel话题
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        
        # 发布传感器信息（值）到sensor_state话题
        self.sensorStatePub = rospy.Publisher('~sensor_state', SensorState, queue_size=5)
        
        # 读取的一些服务
        # A service to position a PWM servo
        rospy.Service('~servo_write', ServoWrite, self.ServoWriteHandler)        
        # A service to read the position of a PWM servo
        rospy.Service('~servo_read', ServoRead, self.ServoReadHandler)      
        # A service to turn set the direction of a digital pin (0 = input, 1 = output)
        rospy.Service('~digital_set_direction', DigitalSetDirection, self.DigitalSetDirectionHandler)        
        # A service to turn a digital sensor on or off
        rospy.Service('~digital_write', DigitalWrite, self.DigitalWriteHandler)       
        # A service to read the value of a digital sensor
        rospy.Service('~digital_read', DigitalRead, self.DigitalReadHandler) 
        # A service to set pwm values for the pins
        rospy.Service('~analog_write', AnalogWrite, self.AnalogWriteHandler)        
        # A service to read the value of an analog sensor
        rospy.Service('~analog_read', AnalogRead, self.AnalogReadHandler)

        # 初始化ARDUINO控制器
        self.controller = Arduino(self.port, self.baud, self.timeout)       
        # 连接
        self.controller.connect()        
        rospy.loginfo("Connected to Arduino on port " + self.port + " at " + str(self.baud) + " baud")
     
        # 创建一个锁对象LockType，使多个线程同步访问共享资源。  多线程
        mutex = thread.allocate_lock()

        # 初始化传感器列表
        self.mySensors = list()
        
        sensor_params = rospy.get_param("~sensors", dict({}))
        
        for name, params in sensor_params.iteritems():
            # Set the direction to input if not specified
            try:
                params['direction']
            except:
                params['direction'] = 'input'
                
            if params['type'] == "Ping":# ping传感器？
                sensor = Ping(self.controller, name, params['pin'], params['rate'], self.base_frame)
            elif params['type'] == "GP2D12":
                sensor = GP2D12(self.controller, name, params['pin'], params['rate'], self.base_frame)
            elif params['type'] == 'Digital':
                sensor = DigitalSensor(self.controller, name, params['pin'], params['rate'], self.base_frame, direction=params['direction'])
            elif params['type'] == 'Analog':
                sensor = AnalogSensor(self.controller, name, params['pin'], params['rate'], self.base_frame, direction=params['direction'])
            elif params['type'] == 'PololuMotorCurrent':
                sensor = PololuMotorCurrent(self.controller, name, params['pin'], params['rate'], self.base_frame)
            elif params['type'] == 'PhidgetsVoltage':
                sensor = PhidgetsVoltage(self.controller, name, params['pin'], params['rate'], self.base_frame)
            elif params['type'] == 'PhidgetsCurrent':
                sensor = PhidgetsCurrent(self.controller, name, params['pin'], params['rate'], self.base_frame)
                
# 超声波传感器                
#                if params['type'] == "MaxEZ1":
#                    self.sensors[len(self.sensors)]['trigger_pin'] = params['trigger_pin']
#                    self.sensors[len(self.sensors)]['output_pin'] = params['output_pin']
            try:
                self.mySensors.append(sensor)
                rospy.loginfo(name + " " + str(params) + " published on topic " + rospy.get_name() + "/sensor/" + name)
            except:
                rospy.logerr("Sensor type " + str(params['type']) + " not recognized.")
              
        # 初始化基本控制器
        if self.use_base_controller:
            self.myBaseController = BaseController(self.controller, self.base_frame, self.name + "_base_controller")
    
        # 发布传感器信息 以及 下发速度等控制命令
        while not rospy.is_shutdown():
            for sensor in self.mySensors:
                mutex.acquire()#开启多线程
                sensor.poll()  # 
                mutex.release()# 关闭多线程
                    
            if self.use_base_controller:
                mutex.acquire()
                self.myBaseController.poll()# 得到轮子当前的速度，发布里程记信息到 话题上，获取cmd_cel话题上的速度命令解析后得到轮子的速度，简单单限值处理后下发到下位机
                mutex.release()
            
            # 在 ~sensor_state 话题上发布所有使用的传感器的数据信息
            now = rospy.Time.now()           
            if now > self.t_next_sensors:
                msg = SensorState()
                msg.header.frame_id = self.base_frame
                msg.header.stamp = now
                for i in range(len(self.mySensors)):
                    msg.name.append(self.mySensors[i].name) #列表
                    msg.value.append(self.mySensors[i].value)
                try:
                    self.sensorStatePub.publish(msg) # 发布传感器信息
                except:
                    pass
                
                self.t_next_sensors = now + self.t_delta_sensors
            
            r.sleep()
    
    # Service callback functions
    def ServoWriteHandler(self, req):
        self.controller.servo_write(req.id, req.value)
        return ServoWriteResponse()
    
    def ServoReadHandler(self, req):
        pos = self.controller.servo_read(req.id)
        return ServoReadResponse(pos)
    
    def DigitalSetDirectionHandler(self, req):
        self.controller.pin_mode(req.pin, req.direction)
        return DigitalSetDirectionResponse()
    
    def DigitalWriteHandler(self, req):
        self.controller.digital_write(req.pin, req.value)
        return DigitalWriteResponse()
    
    def DigitalReadHandler(self, req):
        value = self.controller.digital_read(req.pin)
        return DigitalReadResponse(value)
              
    def AnalogWriteHandler(self, req):
        self.controller.analog_write(req.pin, req.value)
        return AnalogWriteResponse()
    
    def AnalogReadHandler(self, req):
        value = self.controller.analog_read(req.pin)
        return AnalogReadResponse(value)
 
    def shutdown(self):
        rospy.loginfo("Shutting down Arduino Node...")

        # Stop the robot
        try:
            rospy.loginfo("Stopping the robot...")
            self.cmd_vel_pub.Publish(Twist())
            rospy.sleep(2)
        except:
            pass
        
        # Close the serial port
        try:
            self.controller.close()
        except:
            pass
        finally:
            rospy.loginfo("Serial port closed.")
            os._exit(0)

if __name__ == '__main__':
    try:
        myArduino = ArduinoROS()
    except SerialException:
        rospy.logerr("Serial exception trying to open port.")
        os._exit(0)

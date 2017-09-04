#-*- coding:utf-8 -*-
#!/usr/bin/python
"""
    Sensor class for the arudino_python package
    
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

import roslib; roslib.load_manifest('ros_arduino_python')
import rospy
from sensor_msgs.msg import Range
from ros_arduino_msgs.msg import *

LOW = 0
HIGH = 1

INPUT = 0
OUTPUT = 1
    
class MessageType:
    ANALOG = 0
    DIGITAL = 1
    RANGE = 2
    FLOAT = 3
    INT = 4
    BOOL = 5

# 传感器的通用的定义
class Sensor(object):
    def __init__(self, controller, name, pin, rate, frame_id, direction="input", **kwargs):
        self.controller = controller
        self.name = name
        self.pin = pin
        self.rate = rate
        self.direction = direction

        self.frame_id = frame_id
        self.value = None
        
        self.t_delta = rospy.Duration(1.0 / self.rate) # 读取周期
        self.t_next = rospy.Time.now() + self.t_delta  # 下次读取时间
    
    def poll(self):
        now = rospy.Time.now()
        if now > self.t_next:
            if self.direction == "input":         # 输入（采样）
                try:
                    self.value = self.read_value()# 当前读取的值
                except:
                    return
            else:
                try:
                    self.ack = self.write_value()# 输出（驱动）
                except:
                    return          
    
            # For range sensors, assign the value to the range message field
            if self.message_type == MessageType.RANGE:# 对于测量范围 的传感器
                self.msg.range = self.value
            else:
                self.msg.value = self.value

            # 增加时间戳，为了发布信息 Add a timestamp and publish the message
            self.msg.header.stamp = rospy.Time.now()
            self.pub.publish(self.msg)
            
            self.t_next = now + self.t_delta #下次测量时间
            
# 模拟量的传感器    
class AnalogSensor(Sensor):
    def __init__(self, *args, **kwargs):
        super(AnalogSensor, self).__init__(*args, **kwargs)
                
        self.message_type = MessageType.ANALOG
        
        self.msg = Analog()
        self.msg.header.frame_id = self.frame_id
        
        self.pub = rospy.Publisher("~sensor/" + self.name, Analog, queue_size=5)
        
        if self.direction == "output":
            self.controller.pin_mode(self.pin, OUTPUT) # 输出
        else:
            self.controller.pin_mode(self.pin, INPUT)  # 输入

        self.value = LOW
        
    def read_value(self):
        return self.controller.analog_read(self.pin)
    
    def write_value(self, value):
        return self.controller.analog_write(self.pin, value)

# 模拟量浮点型传感器 
class AnalogFloatSensor(Sensor):
    def __init__(self, *args, **kwargs):
        super(AnalogFloatSensor, self).__init__(*args, **kwargs)
                
        self.message_type = MessageType.ANALOG
        
        self.msg = AnalogFloat()
        self.msg.header.frame_id = self.frame_id
        
        self.pub = rospy.Publisher("~sensor/" + self.name, AnalogFloat, queue_size=5)
        
        if self.direction == "output":
            self.controller.pin_mode(self.pin, OUTPUT)
        else:
            self.controller.pin_mode(self.pin, INPUT)

        self.value = LOW
        
    def read_value(self):
        return self.controller.analog_read(self.pin)
    
    def write_value(self, value):
        return self.controller.analog_write(self.pin, value)
    
#数字传感器        
class DigitalSensor(Sensor):
    def __init__(self, *args, **kwargs):
        super(DigitalSensor, self).__init__(*args, **kwargs)
        
        self.message_type = MessageType.BOOL
        
        self.msg = Digital()
        self.msg.header.frame_id = self.frame_id
        
        self.pub = rospy.Publisher("~sensor/" + self.name, Digital, queue_size=5)
        
        if self.direction == "output":
            self.controller.pin_mode(self.pin, OUTPUT)
        else:
            self.controller.pin_mode(self.pin, INPUT)

        self.value = LOW
        
    def read_value(self):
        return self.controller.digital_read(self.pin)
    
    def write_value(self):
        # Alternate HIGH/LOW when writing at a fixed rate
        self.value = not self.value
        return self.controller.digital_write(self.pin, self.value)
    
 # 范围传感器   
class RangeSensor(Sensor):
    def __init__(self, *args, **kwargs):
        super(RangeSensor, self).__init__(*args, **kwargs)
        
        self.message_type = MessageType.RANGE
        
        self.msg = Range()
        self.msg.header.frame_id = self.frame_id
        
        self.pub = rospy.Publisher("~sensor/" + self.name, Range, queue_size=5)
        
    def read_value(self):
        self.msg.header.stamp = rospy.Time.now()
# 声纳（超声波）传感器     
class SonarSensor(RangeSensor):
    def __init__(self, *args, **kwargs):
        super(SonarSensor, self).__init__(*args, **kwargs)
        
        self.msg.radiation_type = Range.ULTRASOUND
        
# 红外线传感器        
class IRSensor(RangeSensor):
    def __init__(self, *args, **kwargs):
        super(IRSensor, self).__init__(*args, **kwargs)
        
        self.msg.radiation_type = Range.INFRARED

#        
class Ping(SonarSensor):
    def __init__(self,*args, **kwargs):
        super(Ping, self).__init__(*args, **kwargs)
                
        self.msg.field_of_view = 0.785398163
        self.msg.min_range = 0.02
        self.msg.max_range = 3.0
        
    def read_value(self):
        # The Arduino Ping code returns the distance in centimeters
        cm = self.controller.ping(self.pin) # Arduino 返回的是 厘米
        
        # Convert it to meters for ROS
        distance = cm / 100.0 # 转化成 ＲＯＳ系统下的距离单位 米
        
        return distance
    
        
class GP2D12(IRSensor):
    def __init__(self, *args, **kwargs):
        super(GP2D12, self).__init__(*args, **kwargs)
        
        self.msg.field_of_view = 0.001
        self.msg.min_range = 0.10
        self.msg.max_range = 0.80
        
    def read_value(self):
        value = self.controller.analog_read(self.pin)
        
        if value <= 3.0:
            return self.msg.max_range
        
        try:
            distance = (6787.0 / (float(value) - 3.0)) - 4.0
        except:
            return self.msg.max_range
            
        # Convert to meters
        distance /= 100.0
        
        # If we get a spurious reading, set it to the max_range
        if distance > self.msg.max_range: distance = self.msg.max_range
        if distance < self.msg.min_range: distance = self.msg.max_range
        
        return distance

# 测量电机 流过的 电流   
class PololuMotorCurrent(AnalogFloatSensor):
    def __init__(self, *args, **kwargs):
        super(PololuMotorCurrent, self).__init__(*args, **kwargs)
        
    def read_value(self):
        # From the Pololu source code
        milliamps = self.controller.analog_read(self.pin) * 34
        return milliamps / 1000.0
    
# 测量电压    
class PhidgetsVoltage(AnalogFloatSensor):
    def __init__(self, *args, **kwargs):
        super(PhidgetsVoltage, self).__init__(*args, **kwargs)
        
    def read_value(self):
        # From the Phidgets documentation
        voltage = 0.06 * (self.controller.analog_read(self.pin) - 500.)
        return voltage
    
class PhidgetsCurrent(AnalogFloatSensor):
    def __init__(self, *args, **kwargs):
        super(PhidgetsCurrent, self).__init__(*args, **kwargs)
        
    def read_value(self):
        # From the Phidgets documentation for the 20 amp DC sensor
        current = 0.05 * (self.controller.analog_read(self.pin) - 500.)
        return current
    
class MaxEZ1Sensor(SonarSensor):
    def __init__(self, *args, **kwargs):
        super(MaxEZ1Sensor, self).__init__(*args, **kwargs)
        
        self.trigger_pin = kwargs['trigger_pin']
        self.output_pin = kwargs['output_pin']
        
        self.msg.field_of_view = 0.785398163
        self.msg.min_range = 0.02
        self.msg.max_range = 3.0
        
    def read_value(self):
        return self.controller.get_MaxEZ1(self.trigger_pin, self.output_pin)

            
if __name__ == '__main__':
    myController = Controller()
    mySensor = SonarSensor(myController, "My Sonar", type=Type.PING, pin=0, rate=10)
            
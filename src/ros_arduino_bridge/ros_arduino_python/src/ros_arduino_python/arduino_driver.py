#-*- coding:utf-8 -*-
#!/usr/bin/python
"""
    A Python driver for the Arduino microcontroller running the
    ROSArduinoBridge firmware.
    
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

import thread
from math import pi as PI, degrees, radians
import os
import time
import sys, traceback
from serial.serialutil import SerialException
from serial import Serial

SERVO_MAX = 180
SERVO_MIN = 0

class Arduino:
    ''' Configuration Parameters
    '''    
    N_ANALOG_PORTS = 6     # 六个模拟量读取口
    N_DIGITAL_PORTS = 12   # 12 个 数字IO
    
    def __init__(self, port="/dev/ttyUSB0", baudrate=57600, timeout=0.5): # arduino硬件端口（挂载点）  波特率   等待时间 
        # PID控制 频率
        self.PID_RATE = 30 # Do not change this!  It is a fixed property of the Arduino PID controller.
        self.PID_INTERVAL = 1000 / 30 #控制周期毫秒
        
        self.port = port           # 端口
        self.baudrate = baudrate   # 波特率
        self.timeout = timeout     # 等待时间
        self.encoder_count = 0     # 编码器计数
        self.writeTimeout = timeout
        self.interCharTimeout = timeout / 30.
    
        # Keep things thread safe  同步时间
        self.mutex = thread.allocate_lock()
            
        # 一个存储模拟量ADC口数据的数组
        self.analog_sensor_cache = [None] * self.N_ANALOG_PORTS
        
        # 一个存储数字量io口数据的数组
        self.digital_sensor_cache = [None] * self.N_DIGITAL_PORTS
        
    # 串口链接下位机器
    def connect(self):
        try:
            print "Connecting to Arduino on port", self.port, "..."
            self.port = Serial(port=self.port, baudrate=self.baudrate, timeout=self.timeout, writeTimeout=self.writeTimeout)
            # 等待 arduino硬件响应
            time.sleep(1)
            test = self.get_baud()   # 硬件下位机 波特率
            if test != self.baudrate:
                time.sleep(1)
                test = self.get_baud()   
                if test != self.baudrate:
                    raise SerialException
            print "Connected at", self.baudrate
            print "Arduino is ready."

        except SerialException:
            print "Serial Exception:"
            print sys.exc_info()
            print "Traceback follows:"
            traceback.print_exc(file=sys.stdout)
            print "Cannot connect to Arduino!"
            os._exit(1)
    # 打开串口
    def open(self): 
        ''' Open the serial port.
        '''
        self.port.open()
    # 关闭串口
    def close(self): 
        ''' Close the serial port.
        '''
        self.port.close() 
    # 通过串口发送数据
    def send(self, cmd):
        ''' This command should not be used on its own: it is called by the execute commands
            below in a thread safe manner.
        '''
        self.port.write(cmd + '\r')
    # 通过串口接收数据
    def recv(self, timeout=0.5):
        timeout = min(timeout, self.timeout)
        ''' This command should not be used on its own: it is called by the execute commands   
            below in a thread safe manner.  Note: we use read() instead of readline() since
            readline() tends to return garbage characters（无用的信息） from the Arduino
        '''
        c = ''     # 读取的中间变量
        value = ''
        attempts = 0 # 读取计数
        while c != '\r': # 
            c = self.port.read(1)
            value += c
            attempts += 1
            if attempts * self.interCharTimeout > timeout:
                return None

        value = value.strip('\r')# 去除 回车

        return value
    # 检测        
    def recv_ack(self):
        ''' This command should not be used on its own: it is called by the execute commands
            below in a thread safe manner.
        '''
        ack = self.recv(self.timeout)
        return ack == 'OK'
    # 初始化
    def recv_int(self):
        ''' This command should not be used on its own: it is called by the execute commands
            below in a thread safe manner.
        '''
        value = self.recv(self.timeout)
        try:
            return int(value)
        except:
            return None
    # 读取 数组
    def recv_array(self):
        ''' This command should not be used on its own: it is called by the execute commands
            below in a thread safe manner.
        '''
        try:
            values = self.recv(self.timeout * self.N_ANALOG_PORTS).split()
            return map(int, values)
        except:
            return []
        
    # 执行指令
    def execute(self, cmd):
        ''' Thread safe execution of "cmd" on the Arduino returning a single integer value.
        '''
        self.mutex.acquire()
        
        try:
            self.port.flushInput()
        except:
            pass
        
        ntries = 1
        attempts = 0
        
        try:
            self.port.write(cmd + '\r')     # 写指令
            value = self.recv(self.timeout) # 读 回应
            # 确保指令正确发送
            while attempts < ntries and (value == '' or value == 'Invalid Command' or value == None):
                try:
                    self.port.flushInput()
                    self.port.write(cmd + '\r')
                    value = self.recv(self.timeout)
                except:
                    print "Exception executing command: " + cmd
                attempts += 1
        except:
            self.mutex.release()
            print "Exception executing command: " + cmd
            value = None
        
        self.mutex.release()
        return int(value)
    
    # 执行 指令数组
    def execute_array(self, cmd):
        ''' Thread safe execution of "cmd" on the Arduino returning an array.
        '''
        self.mutex.acquire()
        
        try:
            self.port.flushInput()
        except:
            pass
        
        ntries = 1
        attempts = 0
        
        try:
            self.port.write(cmd + '\r')
            values = self.recv_array()
            while attempts < ntries and (values == '' or values == 'Invalid Command' or values == [] or values == None):
                try:
                    self.port.flushInput()
                    self.port.write(cmd + '\r')
                    values = self.recv_array()
                except:
                    print("Exception executing command: " + cmd)
                attempts += 1
        except:
            self.mutex.release()
            print "Exception executing command: " + cmd
            raise SerialException
            return []
        
        try:
            values = map(int, values)
        except:
            values = []

        self.mutex.release()
        return values
        
    def execute_ack(self, cmd):
        ''' Thread safe execution of "cmd" on the Arduino returning True if response is ACK.
        '''
        self.mutex.acquire()
        
        try:
            self.port.flushInput()
        except:
            pass
        
        ntries = 1
        attempts = 0
        
        try:
            self.port.write(cmd + '\r')
            ack = self.recv(self.timeout)
            while attempts < ntries and (ack == '' or ack == 'Invalid Command' or ack == None):
                try:
                    self.port.flushInput()
                    self.port.write(cmd + '\r')
                    ack = self.recv(self.timeout)
                except:
                    print "Exception executing command: " + cmd
            attempts += 1
        except:
            self.mutex.release()
            print "execute_ack exception when executing", cmd
            print sys.exc_info()
            return 0
        
        self.mutex.release()
        return ack == 'OK'
    
    # 更新pid参数（向下位机发送）
    def update_pid(self, Kp, Kd, Ki, Ko):
        ''' Set the PID parameters on the Arduino
        '''
        print "Updating PID parameters"
        cmd = 'u ' + str(Kp) + ':' + str(Kd) + ':' + str(Ki) + ':' + str(Ko)
        self.execute_ack(cmd)
        
    # 获取波特率
    def get_baud(self):
        ''' Get the current baud rate on the serial port.
        '''
        try:
            return int(self.execute('b'));
        except:
            return None
        
    # 得到 编码器计数
    def get_encoder_counts(self):
        values = self.execute_array('e')
        if len(values) != 2: # 长度应该为两个值
            print "Encoder count was not 2"
            raise SerialException
            return None
        else:
            return values
    # 重置 编码器
    def reset_encoders(self):
        ''' Reset the encoder counts to 0
        '''
        return self.execute_ack('r')                      # reset 标志
    
    #  发送左右轮子速度指令 
    def drive(self, right, left):
        ''' Speeds are given in encoder ticks per PID interval
        '''
        return self.execute_ack('m %d %d' %(right, left)) # motor 电机速度 标志
    
    def drive_m_per_s(self, right, left):
        ''' Set the motor speeds in meters per second.
        '''
        left_revs_per_second = float(left) / (self.wheel_diameter * PI)   # 轮子线速度/周长  得到转速  
        right_revs_per_second = float(right) / (self.wheel_diameter * PI) # 

        left_ticks_per_loop = int(left_revs_per_second * self.encoder_resolution * self.PID_INTERVAL * self.gear_reduction) # 转速* 分辨率 * 时间 * 齿轮 分辨率 
        right_ticks_per_loop  = int(right_revs_per_second * self.encoder_resolution * self.PID_INTERVAL * self.gear_reduction)

        self.drive(right_ticks_per_loop , left_ticks_per_loop )
        
    # 停止   
    def stop(self):
        ''' Stop both motors.
        '''
        self.drive(0, 0)
    # 读取 管脚 模拟量的值       
    def analog_read(self, pin):
        return self.execute('a %d' %pin) # analog 标志 + 管教编号
    # 模拟量管脚 写入
    def analog_write(self, pin, value):
        return self.execute_ack('x %d %d' %(pin, value))# 
    # 数字量管脚读取
    def digital_read(self, pin):
        return self.execute('d %d' %pin)
    # 数字量管脚写入
    def digital_write(self, pin, value):
        return self.execute_ack('w %d %d' %(pin, value))
    # 设置管脚模式（0输入，1输出）
    def pin_mode(self, pin, mode):
        return self.execute_ack('c %d %d' %(pin, mode))
 
   
    # 设置 舵机旋转角度，需要转换成 角度单位发送（给的pos是弧度制）
    def servo_write(self, id, pos):
        ''' Usage: servo_write(id, pos)
            Position is given in radians and converted to degrees before sending
        '''        
        return self.execute_ack('s %d %d' %(id, min(SERVO_MAX, max(SERVO_MIN, degrees(pos)))))
    
    # 读取 舵机当前的角度（返回弧度制）
    def servo_read(self, id):
        ''' Usage: servo_read(id)
            The returned position is converted from degrees to radians
        '''        
        return radians(self.execute('t %d' %id))


    def ping(self, pin):
        ''' The srf05/Ping command queries an SRF05/Ping sonar sensor
            connected to the General Purpose I/O line pinId for a distance,
            and returns the range in cm.  Sonar distance resolution is integer based.
        '''
        return self.execute('p %d' %pin);
     # 超声波传感器
#    def get_maxez1(self, triggerPin, outputPin):
#        ''' The maxez1 command queries a Maxbotix MaxSonar-EZ1 sonar
#            sensor connected to the General Purpose I/O lines, triggerPin, and
#            outputPin, for a distance, and returns it in Centimeters. NOTE: MAKE
#            SURE there's nothing directly in front of the MaxSonar-EZ1 upon
#            power up, otherwise it wont range correctly for object less than 6
#            inches away! The sensor reading defaults to use English units
#            (inches). The sensor distance resolution is integer based. Also, the
#            maxsonar trigger pin is RX, and the echo pin is PW.
#        '''
#        return self.execute('z %d %d' %(triggerPin, outputPin)) 
 

""" Basic test for connectivity """
if __name__ == "__main__":
    # 端口设置
    if os.name == "posix":
        portName = "/dev/ttyACM0"
    else:
        portName = "COM43" # Windows style COM port.
    # 串口通信波特率设置  
    baudRate = 57600
    # 设置Arduino参数
    myArduino = Arduino(port=portName, baudrate=baudRate, timeout=0.5)
    # 连接Arduino板子
    myArduino.connect()
     
    print "Sleeping for 1 second..."
    time.sleep(1)   
    
    print "Reading on analog port 0", myArduino.analog_read(0)
    print "Reading on digital port 0", myArduino.digital_read(0)
    print "Blinking the LED 3 times"
    for i in range(3):
        myArduino.digital_write(13, 1)
        time.sleep(1.0)
    #print "Current encoder counts", myArduino.encoders()
    
    print "Connection test successful.",
    
    myArduino.stop()
    myArduino.close()
    
    print "Shutting down Arduino."
    

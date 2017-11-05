#!/usr/bin/env python
# -*- coding:utf-8 -*-
import rospy
import sys
import cv2
import cv2.cv as cv
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class cvBridgeDemo():
    def __init__(self):
        self.node_name = "cv_bridge_demo"
        
        rospy.init_node(self.node_name)
        
        # 关闭
        rospy.on_shutdown(self.cleanup)
        
        # 创建 rgb图像 显示窗口
        self.cv_window_name = self.node_name
        cv.NamedWindow(self.cv_window_name, cv.CV_WINDOW_NORMAL)
        cv.MoveWindow(self.cv_window_name, 25, 75)
        
        # 创建深度图像显示窗口
        cv.NamedWindow("Depth Image", cv.CV_WINDOW_NORMAL)
        cv.MoveWindow("Depth Image", 25, 350)
        
        # 创建 ros 图 到 opencv图像转换 对象
        self.bridge = CvBridge()
        
        # 订阅 深度图像和rgb图像数据发布的话题  以及定义 回调函数
        # the appropriate callbacks
        self.image_sub = rospy.Subscriber("input_rgb_image", Image, self.image_callback, queue_size=1)
        self.depth_sub = rospy.Subscriber("input_depth_image", Image, self.depth_callback, queue_size=1)
        # 登陆信息
        rospy.loginfo("Waiting for image topics...")
        rospy.wait_for_message("input_rgb_image", Image)
        rospy.loginfo("Ready.")

    # 收到rgb图像后的回调函数
    def image_callback(self, ros_image):
        # 转换图像格式到opencv格式
        try:
            frame = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        except CvBridgeError, e:
            print e
        # 转换成 numpy 图像数组格式
        frame = np.array(frame, dtype=np.uint8)
        
        # 简单处理图像数据 颜色 滤波 边缘检测等
        display_image = self.process_image(frame)
                       
        # 显示图像
        cv2.imshow(self.node_name, display_image)
        
        # 检测键盘按键事件
        self.keystroke = cv2.waitKey(5)
        if self.keystroke != -1:
            cc = chr(self.keystroke & 255).lower()
            if cc == 'q':
                # The user has press the q key, so exit
                rospy.signal_shutdown("User hit q key to quit.")

    # 收到深度图像后的回调函数            
    def depth_callback(self, ros_image):
        # 转换图像格式到opencv格式
        try:
            # Convert the depth image using the default passthrough encoding
            depth_image = self.bridge.imgmsg_to_cv2(ros_image, "passthrough")
        except CvBridgeError, e:
            print e

        # 转换成 numpy 图像数组格式
        depth_array = np.array(depth_image, dtype=np.float32)
                
        # 深度图像 数据 正则化到 二值图像
        cv2.normalize(depth_array, depth_array, 0, 1, cv2.NORM_MINMAX)
        
        # 深度图像处理
        depth_display_image = self.process_depth_image(depth_array)
    
        # 现实结果
        cv2.imshow("Depth Image", depth_display_image)

    # rgb图像处理      
    def process_image(self, frame):
        # 转化成灰度图像
        grey = cv2.cvtColor(frame, cv.CV_BGR2GRAY)
        
        # 均值滤波
        grey = cv2.blur(grey, (7, 7))
        
        # Canny 边缘检测
        edges = cv2.Canny(grey, 15.0, 30.0)
        
        return edges
    
    # 深度图像处理
    def process_depth_image(self, frame):
        # 这里直接返回原图   可做其他处理
        return frame

    # 关闭节点 销毁所有 窗口
    def cleanup(self):
        print "Shutting down vision node."
        cv2.destroyAllWindows()   

# 主函数    
def main(args):       
    try:
        cvBridgeDemo()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down vision node."
        cv.DestroyAllWindows()

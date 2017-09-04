#!/usr/bin/env python
import rospy
from std_msgs.msg import String

#订阅者 的回调函数
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

#订阅者   
def listener():
    #初始化ros系统  注册节点 hello_Ros_listener 匿名名称  即可同时执行多个
    rospy.init_node('hello_Ros_listener', anonymous=True)
    #订阅 pubHello_ros 
    rospy.Subscriber("pubHello_ros", String, callback)
    
if __name__ == '__main__':
    listener()
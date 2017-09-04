 #!/usr/bin/env python
 # license removed for brevity
import rospy
from std_msgs.msg import String

 
def talker():
    #初始化ros系统  注册节点 talker  匿名名称  即可同时执行多个
    rospy.init_node('pub_hello_talker', anonymous=True)
    #发布到话题 pubHello_ros  数据类型 字符串 缓存队列大小10
    pub = rospy.Publisher('pubHello_ros', String, queue_size=10)
    #发布频率
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello ros at %s" % rospy.get_time()
        #输出到日志 和 屏幕
        rospy.loginfo(hello_str)
        #在话题 上发布消息
        pub.publish(hello_str)
        #休息
        rate.sleep()
 
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
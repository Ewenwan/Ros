#!/usr/bin/env python
#-*- coding:utf-8 -*-


import rospy
import smach
import smach_ros

# Foo状态 define state Foo
class Foo(smach.State):
    #初始化函数
    def __init__(self):
	# 该状态 有两个输出 'outcome1' 和'outcome2'
        smach.State.__init__(self, outcomes=['outcome1','outcome2'],
				   input_keys=['foo_counter_in'], #两个参数就是状态的输入输出数据
                                   output_keys=['foo_counter_out'])
        # self.counter = 0
    #执行函数
    def execute(self, userdata):# userdata参数， 就是存储状态之间需要传递数据的容器
        rospy.loginfo('Executing state FOO')
        #if self.counter < 3:
            #self.counter += 1
        if userdata.foo_counter_in < 3:
	    userdata.foo_counter_out = userdata.foo_counter_in + 1
            return 'outcome1' #前三次输出 'outcome1'
        else:
            return 'outcome2' #后一次输出 'outcome2'

# Bar 状态 define state Bar
class Bar(smach.State):
    def __init__(self):
	# 该状态 有一个输出  'outcome2'
        smach.State.__init__(self, outcomes=['outcome2'],
			           input_keys=['bar_counter_in'])

    def execute(self, userdata):
        rospy.loginfo('Executing state BAR')
	rospy.loginfo('Counter = %d'%userdata.bar_counter_in)
        return 'outcome2'

# main
def main():
    #初始化节点
    rospy.init_node('smach_example_state_machine_data')

    # Create a SMACH state machine
    # 使用StateMachine创建一个状态机
    # 并且指定状态机执行结束后的最终输出值有两个：outcome4 
    sm = smach.StateMachine(outcomes=['outcome4'])
    # 定义了状态之间传递数据的变量sm_counter 所有其他变量都转换成这边变量进行传递
    sm.userdata.sm_counter = 0#用户数据初始化为0
    # SMACH状态机是一个容器，我们可以使用add方法添加需要的状态到状态机容器当中，
    # 同时需要设置状态之间的跳转关系
    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('FOO', Foo(),#状态 
			       # 根据输出 执行的跳转
                               transitions={'outcome1':'BAR', # 输出为 'outcome1' 时 跳转到BAR状态
                                            'outcome2':'outcome4'},#输出为 'outcome2'结束，输出结果 'outcome4'
			       remapping={'foo_counter_in':'sm_counter', # 输入输出数据 变量 重映射
            				  'foo_counter_out':'sm_counter'})
        smach.StateMachine.add('BAR', Bar(), #状态 
                               transitions={'outcome2':'FOO'},# 输出为 'outcome2' 跳转到 FOO
			       remapping={'bar_counter_in':'sm_counter'})
    
    # Create and start the introspection server
    # 可视化状态机服务器  可是使用 rosrun smach_viewer smach_viewer.py 查看
    # 第一个参数是服务器的名字，可以根据需要自由给定；
    # 第二个参数是所要监测的状态机；
    # 第三个参数代表状态机的层级，因为SMACH状态机支持嵌套，状态内部还可以有自己的状态机。
    sis = smach_ros.IntrospectionServer('my_smach_introspection_server', sm, '/SM_ROOT')
    sis.start()
    
    # Execute SMACH plan
    # 执行状态机
    outcome = sm.execute()
    
    # Wait for ctrl-c to stop the application
    # 等待状态机结束
    rospy.spin()
    sis.stop()#停止 可视化状态机服务器

if __name__ == '__main__':
	main()

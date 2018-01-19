#!/usr/bin/env python
#-*- coding:utf-8 -*-

'''
SMACH中的状态机是容器，支持嵌套功能，也就是说在状态机中还可以实现一个内部的状态机。
'''
import rospy
import smach
import smach_ros

# Foo状态 define state Foo
class Foo(smach.State):
    #初始化函数
    def __init__(self):
	# 该状态 有两个输出 'outcome1' 和'outcome2'
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0
    #执行函数
    def execute(self, userdata):
        rospy.loginfo('Executing state FOO')
        if self.counter < 3:
            self.counter += 1
            return 'outcome1' #前三次输出 'outcome1'
        else:
            return 'outcome2' #后一次输出 'outcome2'

# Bar 状态 define state Bar
class Bar(smach.State):
    def __init__(self):
	# 该状态 有一个输出  'outcome2'
        smach.State.__init__(self, outcomes=['outcome2'])

    def execute(self, userdata):
        rospy.loginfo('Executing state BAR')
        return 'outcome2'

# Bas状态 define state Bas
class Bas(smach.State):
    def __init__(self):
        # 该状态 有一个输出  'outcome3'
        smach.State.__init__(self, outcomes=['outcome3'])

    def execute(self, userdata):
        rospy.loginfo('Executing state BAS')
        return 'outcome3'

# main
def main():
    #初始化节点
    rospy.init_node('smach_example_state_machine_nest')

    # Create a SMACH state machine
    # 使用StateMachine创建一个状态机 
    # 最外层状态机  最终输出的结果 'outcome5'
    sm_top = smach.StateMachine(outcomes=['outcome5'])
    # 打开容器
    with sm_top:
        # 添加一个状态
        smach.StateMachine.add('BAS', Bas(),#状态 
			       # 根据输出 执行的跳转
                               transitions={'outcome3':'SUB_SM'})#跳转到SUM_AM 子状态机 
	# 添加一个 子 状态机 sm_sub
        # 创建子状态机
        sm_sub = smach.StateMachine(outcomes=['outcome4']) #输出'outcome4'
	with sm_sub:#向子状态机 容器内 添加状态
	    smach.StateMachine.add('FOO', Foo(),#状态 
				       # 根据输出 执行的跳转
		                       transitions={'outcome1':'BAR', # 输出为 'outcome1' 时 跳转到BAR状态
		                                    'outcome2':'outcome4'})#输出为 'outcome2'结束，输出结果 'outcome4'
	    smach.StateMachine.add('BAR', Bar(), #状态 
		                       transitions={'outcome2':'FOO'})# 输出为 'outcome2' 跳转到 FOO    

        smach.StateMachine.add('SUB_SM', sm_sub,
                               transitions={'outcome4':'outcome5'})	



    # Create and start the introspection server
    # 可视化状态机服务器  可是使用 rosrun smach_viewer smach_viewer.py 查看
    # 第一个参数是服务器的名字，可以根据需要自由给定；
    # 第二个参数是所要监测的状态机；
    # 第三个参数代表状态机的层级，因为SMACH状态机支持嵌套，状态内部还可以有自己的状态机。
    sis = smach_ros.IntrospectionServer('my_smach_introspection_server', sm_top, '/SM_ROOT')
    sis.start()
    
    # Execute SMACH plan
    # 执行状态机
    outcome = sm_top.execute()
    
    # Wait for ctrl-c to stop the application
    # 等待状态机结束
    rospy.spin()
    sis.stop()#停止 可视化状态机服务器

if __name__ == '__main__':
	main()

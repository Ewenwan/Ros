#!/usr/bin/env python
#-*- coding:utf-8 -*-

'''
SMACH还支持多个状态并列运行
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
        smach.State.__init__(self, outcomes=['outcome1'])

    def execute(self, userdata):
        rospy.loginfo('Executing state BAR')
        return 'outcome1'

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
    rospy.init_node('smach_example_state_machine_con')

    # Create a SMACH state machine
    # 使用StateMachine创建一个状态机 
    # 最外层状态机  最终输出的结果 'outcome6'
    sm_top = smach.StateMachine(outcomes=['outcome6'])
    # 打开容器
    with sm_top:
        # 添加一个状态
        smach.StateMachine.add('BAS', Bas(),#状态 
                               transitions={'outcome3':'CON_SM'})#跳转到 CON_SM子状态机
 
	# 添加一个 子 状态机 sm_con
        # 创建 可并行的子状态机 同步状态机
        sm_con = smach.Concurrence(outcomes=['outcome4','outcome5'],#子状态机两个输出
                                   default_outcome='outcome4',#默认输出'outcome4' 循环执行该状态机
                                   # 设置了状态机同步运行的状态跳转
                                   outcome_map={'outcome5':# 另一个'outcome5'
                                   { 'FOO':'outcome2',# 当'FOO'输出'outcome2'且'BAR'输出'outcome1'
                                   'BAR':'outcome1'}})# 时 才输出 'outcome5' 跳转到 最外层状态机 
	with sm_con:#使用同步状态机 向子状态机 容器内 添加状态
	    smach.Concurrence.add('FOO', Foo())
	    smach.Concurrence.add('BAR', Bar())
        
	# 添加一个并行的子状态机
        smach.StateMachine.add('CON_SM', sm_con,
                               transitions={'outcome4':'CON_SM',# 子状态机的一个输出 默认转到自己 循环
					    'outcome5':'outcome6'})	



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

SMACH 是状态机的意思，是基于Python实现的一个功能强大且易于扩展的库。
smach本质上并不依赖于ROS，可以用于任意Python项目，
不过在ROS中元功能包executive_smach将smach和ROS很好的集成在了一起，
可以为机器人复杂应用开发提供任务级的状态机框架，
此外元功能包还集成了actionlib和smach_viewer。

# [1简单状态机](1smach_simp.py)         smach.StateMachine()
# [2使用用户数据状态机](2smach_using_date.py) smach.StateMachine()
# [3嵌套状态机](3smach_nesting.py)           smach.StateMachine()
# [4并行状态机](4smach_concurrence.py)  smach.Concurrence()
# 顺序执行状态机 smach.Sequence()
# 循环执行状态机 smach.Iterator()

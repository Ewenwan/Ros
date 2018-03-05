# 源文件

[1 工件位置发布节点vision_node.cpp](vision_node.cpp)
订阅　fake_ar_publisher　话题消息　打印工件位置

[２ 提供工件位置的服务　ARserver.cpp](vARserver.cpp)
订阅　fake_ar_publisher　话题消息　发布返回工件位置（相对位姿变换）的服务　

[３ 提供笛卡尔规划的服务　descartes_node.cpp](descartes_node.cpp)
针对某关节点的轨迹　　规划出路径　返回　路径上的　点集合

[4 提供笛卡尔规划的服务　ARclient.cpp](ARclient.cpp)
moveit 接口 运动规划 机械臂从当前位置　运动到　工件位置　Descartes  笛卡尔 轨迹规划接口　使用轨迹跟踪执行　action行动执行　工具在工件位置周围执行

cmake_minimum_required(VERSION 2.8.3)

project(rbx1_nav)

find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure)

catkin_python_setup()

generate_dynamic_reconfigure_options(cfg/CalibrateAngular.cfg cfg/CalibrateLinear.cfg)

catkin_package(CATKIN_DEPENDS dynamic_reconfigure)

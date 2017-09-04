cmake_minimum_required(VERSION 2.8.3)
project(rosserial_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation)

add_message_files(FILES
                  Log.msg
                  TopicInfo.msg
                 )

add_service_files(FILES
                  RequestParam.srv
                  RequestMessageInfo.srv
                  RequestServiceInfo.srv
                 )

generate_messages()
catkin_package(CATKIN_DEPENDS message_runtime)

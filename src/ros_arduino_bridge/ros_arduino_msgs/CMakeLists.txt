cmake_minimum_required(VERSION 2.8.3)
project(ros_arduino_msgs)

find_package(catkin REQUIRED COMPONENTS std_msgs message_generation)

add_message_files(FILES
                  AnalogFloat.msg
                  Analog.msg
                  ArduinoConstants.msg
                  Digital.msg
                  SensorState.msg
                 )

add_service_files(FILES
                  DigitalSetDirection.srv
                  DigitalWrite.srv
                  DigitalRead.srv
                  ServoRead.srv
                  ServoWrite.srv
                  AnalogWrite.srv
                  AnalogRead.srv
                 )

generate_messages(   
	DEPENDENCIES  
	std_msgs  
)  

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

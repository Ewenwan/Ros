
(cl:in-package :asdf)

(defsystem "ros_arduino_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "AnalogFloat" :depends-on ("_package_AnalogFloat"))
    (:file "_package_AnalogFloat" :depends-on ("_package"))
    (:file "Analog" :depends-on ("_package_Analog"))
    (:file "_package_Analog" :depends-on ("_package"))
    (:file "Digital" :depends-on ("_package_Digital"))
    (:file "_package_Digital" :depends-on ("_package"))
    (:file "SensorState" :depends-on ("_package_SensorState"))
    (:file "_package_SensorState" :depends-on ("_package"))
    (:file "ArduinoConstants" :depends-on ("_package_ArduinoConstants"))
    (:file "_package_ArduinoConstants" :depends-on ("_package"))
  ))
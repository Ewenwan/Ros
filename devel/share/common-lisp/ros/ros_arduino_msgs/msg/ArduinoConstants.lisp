; Auto-generated. Do not edit!


(cl:in-package ros_arduino_msgs-msg)


;//! \htmlinclude ArduinoConstants.msg.html

(cl:defclass <ArduinoConstants> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass ArduinoConstants (<ArduinoConstants>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ArduinoConstants>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ArduinoConstants)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ros_arduino_msgs-msg:<ArduinoConstants> is deprecated: use ros_arduino_msgs-msg:ArduinoConstants instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<ArduinoConstants>)))
    "Constants for message type '<ArduinoConstants>"
  '((:LOW . 0)
    (:HIGH . 1)
    (:INPUT . 0)
    (:OUTPUT . 1))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'ArduinoConstants)))
    "Constants for message type 'ArduinoConstants"
  '((:LOW . 0)
    (:HIGH . 1)
    (:INPUT . 0)
    (:OUTPUT . 1))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ArduinoConstants>) ostream)
  "Serializes a message object of type '<ArduinoConstants>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ArduinoConstants>) istream)
  "Deserializes a message object of type '<ArduinoConstants>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ArduinoConstants>)))
  "Returns string type for a message object of type '<ArduinoConstants>"
  "ros_arduino_msgs/ArduinoConstants")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ArduinoConstants)))
  "Returns string type for a message object of type 'ArduinoConstants"
  "ros_arduino_msgs/ArduinoConstants")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ArduinoConstants>)))
  "Returns md5sum for a message object of type '<ArduinoConstants>"
  "6ca092be59914d9e8dd11612f0a5c895")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ArduinoConstants)))
  "Returns md5sum for a message object of type 'ArduinoConstants"
  "6ca092be59914d9e8dd11612f0a5c895")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ArduinoConstants>)))
  "Returns full string definition for message of type '<ArduinoConstants>"
  (cl:format cl:nil "# Arduino-style constants~%uint8 LOW=0~%uint8 HIGH=1~%uint8 INPUT=0~%uint8 OUTPUT=1~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ArduinoConstants)))
  "Returns full string definition for message of type 'ArduinoConstants"
  (cl:format cl:nil "# Arduino-style constants~%uint8 LOW=0~%uint8 HIGH=1~%uint8 INPUT=0~%uint8 OUTPUT=1~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ArduinoConstants>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ArduinoConstants>))
  "Converts a ROS message object to a list"
  (cl:list 'ArduinoConstants
))

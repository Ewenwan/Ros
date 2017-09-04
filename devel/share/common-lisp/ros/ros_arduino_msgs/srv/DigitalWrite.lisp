; Auto-generated. Do not edit!


(cl:in-package ros_arduino_msgs-srv)


;//! \htmlinclude DigitalWrite-request.msg.html

(cl:defclass <DigitalWrite-request> (roslisp-msg-protocol:ros-message)
  ((pin
    :reader pin
    :initarg :pin
    :type cl:fixnum
    :initform 0)
   (value
    :reader value
    :initarg :value
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass DigitalWrite-request (<DigitalWrite-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DigitalWrite-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DigitalWrite-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ros_arduino_msgs-srv:<DigitalWrite-request> is deprecated: use ros_arduino_msgs-srv:DigitalWrite-request instead.")))

(cl:ensure-generic-function 'pin-val :lambda-list '(m))
(cl:defmethod pin-val ((m <DigitalWrite-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_arduino_msgs-srv:pin-val is deprecated.  Use ros_arduino_msgs-srv:pin instead.")
  (pin m))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <DigitalWrite-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_arduino_msgs-srv:value-val is deprecated.  Use ros_arduino_msgs-srv:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DigitalWrite-request>) ostream)
  "Serializes a message object of type '<DigitalWrite-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'pin)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'value) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DigitalWrite-request>) istream)
  "Deserializes a message object of type '<DigitalWrite-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'pin)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'value) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DigitalWrite-request>)))
  "Returns string type for a service object of type '<DigitalWrite-request>"
  "ros_arduino_msgs/DigitalWriteRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DigitalWrite-request)))
  "Returns string type for a service object of type 'DigitalWrite-request"
  "ros_arduino_msgs/DigitalWriteRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DigitalWrite-request>)))
  "Returns md5sum for a message object of type '<DigitalWrite-request>"
  "9965f904e6efea32066b0a4a77246056")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DigitalWrite-request)))
  "Returns md5sum for a message object of type 'DigitalWrite-request"
  "9965f904e6efea32066b0a4a77246056")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DigitalWrite-request>)))
  "Returns full string definition for message of type '<DigitalWrite-request>"
  (cl:format cl:nil "uint8 pin~%bool value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DigitalWrite-request)))
  "Returns full string definition for message of type 'DigitalWrite-request"
  (cl:format cl:nil "uint8 pin~%bool value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DigitalWrite-request>))
  (cl:+ 0
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DigitalWrite-request>))
  "Converts a ROS message object to a list"
  (cl:list 'DigitalWrite-request
    (cl:cons ':pin (pin msg))
    (cl:cons ':value (value msg))
))
;//! \htmlinclude DigitalWrite-response.msg.html

(cl:defclass <DigitalWrite-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass DigitalWrite-response (<DigitalWrite-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DigitalWrite-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DigitalWrite-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ros_arduino_msgs-srv:<DigitalWrite-response> is deprecated: use ros_arduino_msgs-srv:DigitalWrite-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DigitalWrite-response>) ostream)
  "Serializes a message object of type '<DigitalWrite-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DigitalWrite-response>) istream)
  "Deserializes a message object of type '<DigitalWrite-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DigitalWrite-response>)))
  "Returns string type for a service object of type '<DigitalWrite-response>"
  "ros_arduino_msgs/DigitalWriteResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DigitalWrite-response)))
  "Returns string type for a service object of type 'DigitalWrite-response"
  "ros_arduino_msgs/DigitalWriteResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DigitalWrite-response>)))
  "Returns md5sum for a message object of type '<DigitalWrite-response>"
  "9965f904e6efea32066b0a4a77246056")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DigitalWrite-response)))
  "Returns md5sum for a message object of type 'DigitalWrite-response"
  "9965f904e6efea32066b0a4a77246056")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DigitalWrite-response>)))
  "Returns full string definition for message of type '<DigitalWrite-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DigitalWrite-response)))
  "Returns full string definition for message of type 'DigitalWrite-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DigitalWrite-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DigitalWrite-response>))
  "Converts a ROS message object to a list"
  (cl:list 'DigitalWrite-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'DigitalWrite)))
  'DigitalWrite-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'DigitalWrite)))
  'DigitalWrite-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DigitalWrite)))
  "Returns string type for a service object of type '<DigitalWrite>"
  "ros_arduino_msgs/DigitalWrite")
; Auto-generated. Do not edit!


(cl:in-package ros_arduino_msgs-srv)


;//! \htmlinclude AnalogWrite-request.msg.html

(cl:defclass <AnalogWrite-request> (roslisp-msg-protocol:ros-message)
  ((pin
    :reader pin
    :initarg :pin
    :type cl:fixnum
    :initform 0)
   (value
    :reader value
    :initarg :value
    :type cl:fixnum
    :initform 0))
)

(cl:defclass AnalogWrite-request (<AnalogWrite-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AnalogWrite-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AnalogWrite-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ros_arduino_msgs-srv:<AnalogWrite-request> is deprecated: use ros_arduino_msgs-srv:AnalogWrite-request instead.")))

(cl:ensure-generic-function 'pin-val :lambda-list '(m))
(cl:defmethod pin-val ((m <AnalogWrite-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_arduino_msgs-srv:pin-val is deprecated.  Use ros_arduino_msgs-srv:pin instead.")
  (pin m))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <AnalogWrite-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_arduino_msgs-srv:value-val is deprecated.  Use ros_arduino_msgs-srv:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AnalogWrite-request>) ostream)
  "Serializes a message object of type '<AnalogWrite-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'pin)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'value)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'value)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AnalogWrite-request>) istream)
  "Deserializes a message object of type '<AnalogWrite-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'pin)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'value)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'value)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AnalogWrite-request>)))
  "Returns string type for a service object of type '<AnalogWrite-request>"
  "ros_arduino_msgs/AnalogWriteRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AnalogWrite-request)))
  "Returns string type for a service object of type 'AnalogWrite-request"
  "ros_arduino_msgs/AnalogWriteRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AnalogWrite-request>)))
  "Returns md5sum for a message object of type '<AnalogWrite-request>"
  "8bdf3293d28cac28419ebc4ff41dad0d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AnalogWrite-request)))
  "Returns md5sum for a message object of type 'AnalogWrite-request"
  "8bdf3293d28cac28419ebc4ff41dad0d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AnalogWrite-request>)))
  "Returns full string definition for message of type '<AnalogWrite-request>"
  (cl:format cl:nil "uint8 pin~%uint16 value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AnalogWrite-request)))
  "Returns full string definition for message of type 'AnalogWrite-request"
  (cl:format cl:nil "uint8 pin~%uint16 value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AnalogWrite-request>))
  (cl:+ 0
     1
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AnalogWrite-request>))
  "Converts a ROS message object to a list"
  (cl:list 'AnalogWrite-request
    (cl:cons ':pin (pin msg))
    (cl:cons ':value (value msg))
))
;//! \htmlinclude AnalogWrite-response.msg.html

(cl:defclass <AnalogWrite-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass AnalogWrite-response (<AnalogWrite-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AnalogWrite-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AnalogWrite-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ros_arduino_msgs-srv:<AnalogWrite-response> is deprecated: use ros_arduino_msgs-srv:AnalogWrite-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AnalogWrite-response>) ostream)
  "Serializes a message object of type '<AnalogWrite-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AnalogWrite-response>) istream)
  "Deserializes a message object of type '<AnalogWrite-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AnalogWrite-response>)))
  "Returns string type for a service object of type '<AnalogWrite-response>"
  "ros_arduino_msgs/AnalogWriteResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AnalogWrite-response)))
  "Returns string type for a service object of type 'AnalogWrite-response"
  "ros_arduino_msgs/AnalogWriteResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AnalogWrite-response>)))
  "Returns md5sum for a message object of type '<AnalogWrite-response>"
  "8bdf3293d28cac28419ebc4ff41dad0d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AnalogWrite-response)))
  "Returns md5sum for a message object of type 'AnalogWrite-response"
  "8bdf3293d28cac28419ebc4ff41dad0d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AnalogWrite-response>)))
  "Returns full string definition for message of type '<AnalogWrite-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AnalogWrite-response)))
  "Returns full string definition for message of type 'AnalogWrite-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AnalogWrite-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AnalogWrite-response>))
  "Converts a ROS message object to a list"
  (cl:list 'AnalogWrite-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'AnalogWrite)))
  'AnalogWrite-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'AnalogWrite)))
  'AnalogWrite-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AnalogWrite)))
  "Returns string type for a service object of type '<AnalogWrite>"
  "ros_arduino_msgs/AnalogWrite")
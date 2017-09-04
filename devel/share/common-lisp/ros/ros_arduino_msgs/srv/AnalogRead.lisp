; Auto-generated. Do not edit!


(cl:in-package ros_arduino_msgs-srv)


;//! \htmlinclude AnalogRead-request.msg.html

(cl:defclass <AnalogRead-request> (roslisp-msg-protocol:ros-message)
  ((pin
    :reader pin
    :initarg :pin
    :type cl:fixnum
    :initform 0))
)

(cl:defclass AnalogRead-request (<AnalogRead-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AnalogRead-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AnalogRead-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ros_arduino_msgs-srv:<AnalogRead-request> is deprecated: use ros_arduino_msgs-srv:AnalogRead-request instead.")))

(cl:ensure-generic-function 'pin-val :lambda-list '(m))
(cl:defmethod pin-val ((m <AnalogRead-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_arduino_msgs-srv:pin-val is deprecated.  Use ros_arduino_msgs-srv:pin instead.")
  (pin m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AnalogRead-request>) ostream)
  "Serializes a message object of type '<AnalogRead-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'pin)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AnalogRead-request>) istream)
  "Deserializes a message object of type '<AnalogRead-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'pin)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AnalogRead-request>)))
  "Returns string type for a service object of type '<AnalogRead-request>"
  "ros_arduino_msgs/AnalogReadRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AnalogRead-request)))
  "Returns string type for a service object of type 'AnalogRead-request"
  "ros_arduino_msgs/AnalogReadRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AnalogRead-request>)))
  "Returns md5sum for a message object of type '<AnalogRead-request>"
  "390d2907e6c6cb9c9490e8fab3391260")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AnalogRead-request)))
  "Returns md5sum for a message object of type 'AnalogRead-request"
  "390d2907e6c6cb9c9490e8fab3391260")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AnalogRead-request>)))
  "Returns full string definition for message of type '<AnalogRead-request>"
  (cl:format cl:nil "uint8 pin~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AnalogRead-request)))
  "Returns full string definition for message of type 'AnalogRead-request"
  (cl:format cl:nil "uint8 pin~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AnalogRead-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AnalogRead-request>))
  "Converts a ROS message object to a list"
  (cl:list 'AnalogRead-request
    (cl:cons ':pin (pin msg))
))
;//! \htmlinclude AnalogRead-response.msg.html

(cl:defclass <AnalogRead-response> (roslisp-msg-protocol:ros-message)
  ((value
    :reader value
    :initarg :value
    :type cl:fixnum
    :initform 0))
)

(cl:defclass AnalogRead-response (<AnalogRead-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AnalogRead-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AnalogRead-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ros_arduino_msgs-srv:<AnalogRead-response> is deprecated: use ros_arduino_msgs-srv:AnalogRead-response instead.")))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <AnalogRead-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_arduino_msgs-srv:value-val is deprecated.  Use ros_arduino_msgs-srv:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AnalogRead-response>) ostream)
  "Serializes a message object of type '<AnalogRead-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'value)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'value)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AnalogRead-response>) istream)
  "Deserializes a message object of type '<AnalogRead-response>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'value)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'value)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AnalogRead-response>)))
  "Returns string type for a service object of type '<AnalogRead-response>"
  "ros_arduino_msgs/AnalogReadResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AnalogRead-response)))
  "Returns string type for a service object of type 'AnalogRead-response"
  "ros_arduino_msgs/AnalogReadResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AnalogRead-response>)))
  "Returns md5sum for a message object of type '<AnalogRead-response>"
  "390d2907e6c6cb9c9490e8fab3391260")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AnalogRead-response)))
  "Returns md5sum for a message object of type 'AnalogRead-response"
  "390d2907e6c6cb9c9490e8fab3391260")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AnalogRead-response>)))
  "Returns full string definition for message of type '<AnalogRead-response>"
  (cl:format cl:nil "uint16 value~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AnalogRead-response)))
  "Returns full string definition for message of type 'AnalogRead-response"
  (cl:format cl:nil "uint16 value~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AnalogRead-response>))
  (cl:+ 0
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AnalogRead-response>))
  "Converts a ROS message object to a list"
  (cl:list 'AnalogRead-response
    (cl:cons ':value (value msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'AnalogRead)))
  'AnalogRead-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'AnalogRead)))
  'AnalogRead-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AnalogRead)))
  "Returns string type for a service object of type '<AnalogRead>"
  "ros_arduino_msgs/AnalogRead")
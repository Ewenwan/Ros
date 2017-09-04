; Auto-generated. Do not edit!


(cl:in-package ros_arduino_msgs-srv)


;//! \htmlinclude DigitalRead-request.msg.html

(cl:defclass <DigitalRead-request> (roslisp-msg-protocol:ros-message)
  ((pin
    :reader pin
    :initarg :pin
    :type cl:fixnum
    :initform 0))
)

(cl:defclass DigitalRead-request (<DigitalRead-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DigitalRead-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DigitalRead-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ros_arduino_msgs-srv:<DigitalRead-request> is deprecated: use ros_arduino_msgs-srv:DigitalRead-request instead.")))

(cl:ensure-generic-function 'pin-val :lambda-list '(m))
(cl:defmethod pin-val ((m <DigitalRead-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_arduino_msgs-srv:pin-val is deprecated.  Use ros_arduino_msgs-srv:pin instead.")
  (pin m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DigitalRead-request>) ostream)
  "Serializes a message object of type '<DigitalRead-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'pin)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DigitalRead-request>) istream)
  "Deserializes a message object of type '<DigitalRead-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'pin)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DigitalRead-request>)))
  "Returns string type for a service object of type '<DigitalRead-request>"
  "ros_arduino_msgs/DigitalReadRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DigitalRead-request)))
  "Returns string type for a service object of type 'DigitalRead-request"
  "ros_arduino_msgs/DigitalReadRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DigitalRead-request>)))
  "Returns md5sum for a message object of type '<DigitalRead-request>"
  "78b8839065b88768904414e0b6e384fc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DigitalRead-request)))
  "Returns md5sum for a message object of type 'DigitalRead-request"
  "78b8839065b88768904414e0b6e384fc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DigitalRead-request>)))
  "Returns full string definition for message of type '<DigitalRead-request>"
  (cl:format cl:nil "uint8 pin~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DigitalRead-request)))
  "Returns full string definition for message of type 'DigitalRead-request"
  (cl:format cl:nil "uint8 pin~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DigitalRead-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DigitalRead-request>))
  "Converts a ROS message object to a list"
  (cl:list 'DigitalRead-request
    (cl:cons ':pin (pin msg))
))
;//! \htmlinclude DigitalRead-response.msg.html

(cl:defclass <DigitalRead-response> (roslisp-msg-protocol:ros-message)
  ((value
    :reader value
    :initarg :value
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass DigitalRead-response (<DigitalRead-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DigitalRead-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DigitalRead-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ros_arduino_msgs-srv:<DigitalRead-response> is deprecated: use ros_arduino_msgs-srv:DigitalRead-response instead.")))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <DigitalRead-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_arduino_msgs-srv:value-val is deprecated.  Use ros_arduino_msgs-srv:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DigitalRead-response>) ostream)
  "Serializes a message object of type '<DigitalRead-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'value) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DigitalRead-response>) istream)
  "Deserializes a message object of type '<DigitalRead-response>"
    (cl:setf (cl:slot-value msg 'value) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DigitalRead-response>)))
  "Returns string type for a service object of type '<DigitalRead-response>"
  "ros_arduino_msgs/DigitalReadResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DigitalRead-response)))
  "Returns string type for a service object of type 'DigitalRead-response"
  "ros_arduino_msgs/DigitalReadResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DigitalRead-response>)))
  "Returns md5sum for a message object of type '<DigitalRead-response>"
  "78b8839065b88768904414e0b6e384fc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DigitalRead-response)))
  "Returns md5sum for a message object of type 'DigitalRead-response"
  "78b8839065b88768904414e0b6e384fc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DigitalRead-response>)))
  "Returns full string definition for message of type '<DigitalRead-response>"
  (cl:format cl:nil "bool value~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DigitalRead-response)))
  "Returns full string definition for message of type 'DigitalRead-response"
  (cl:format cl:nil "bool value~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DigitalRead-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DigitalRead-response>))
  "Converts a ROS message object to a list"
  (cl:list 'DigitalRead-response
    (cl:cons ':value (value msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'DigitalRead)))
  'DigitalRead-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'DigitalRead)))
  'DigitalRead-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DigitalRead)))
  "Returns string type for a service object of type '<DigitalRead>"
  "ros_arduino_msgs/DigitalRead")
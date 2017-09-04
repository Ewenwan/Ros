; Auto-generated. Do not edit!


(cl:in-package ros_arduino_msgs-srv)


;//! \htmlinclude ServoRead-request.msg.html

(cl:defclass <ServoRead-request> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:fixnum
    :initform 0))
)

(cl:defclass ServoRead-request (<ServoRead-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ServoRead-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ServoRead-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ros_arduino_msgs-srv:<ServoRead-request> is deprecated: use ros_arduino_msgs-srv:ServoRead-request instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <ServoRead-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_arduino_msgs-srv:id-val is deprecated.  Use ros_arduino_msgs-srv:id instead.")
  (id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ServoRead-request>) ostream)
  "Serializes a message object of type '<ServoRead-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ServoRead-request>) istream)
  "Deserializes a message object of type '<ServoRead-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ServoRead-request>)))
  "Returns string type for a service object of type '<ServoRead-request>"
  "ros_arduino_msgs/ServoReadRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ServoRead-request)))
  "Returns string type for a service object of type 'ServoRead-request"
  "ros_arduino_msgs/ServoReadRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ServoRead-request>)))
  "Returns md5sum for a message object of type '<ServoRead-request>"
  "6685c94dd9155802f37ed34fb627a83a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ServoRead-request)))
  "Returns md5sum for a message object of type 'ServoRead-request"
  "6685c94dd9155802f37ed34fb627a83a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ServoRead-request>)))
  "Returns full string definition for message of type '<ServoRead-request>"
  (cl:format cl:nil "uint8 id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ServoRead-request)))
  "Returns full string definition for message of type 'ServoRead-request"
  (cl:format cl:nil "uint8 id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ServoRead-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ServoRead-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ServoRead-request
    (cl:cons ':id (id msg))
))
;//! \htmlinclude ServoRead-response.msg.html

(cl:defclass <ServoRead-response> (roslisp-msg-protocol:ros-message)
  ((value
    :reader value
    :initarg :value
    :type cl:float
    :initform 0.0))
)

(cl:defclass ServoRead-response (<ServoRead-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ServoRead-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ServoRead-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ros_arduino_msgs-srv:<ServoRead-response> is deprecated: use ros_arduino_msgs-srv:ServoRead-response instead.")))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <ServoRead-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_arduino_msgs-srv:value-val is deprecated.  Use ros_arduino_msgs-srv:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ServoRead-response>) ostream)
  "Serializes a message object of type '<ServoRead-response>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'value))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ServoRead-response>) istream)
  "Deserializes a message object of type '<ServoRead-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'value) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ServoRead-response>)))
  "Returns string type for a service object of type '<ServoRead-response>"
  "ros_arduino_msgs/ServoReadResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ServoRead-response)))
  "Returns string type for a service object of type 'ServoRead-response"
  "ros_arduino_msgs/ServoReadResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ServoRead-response>)))
  "Returns md5sum for a message object of type '<ServoRead-response>"
  "6685c94dd9155802f37ed34fb627a83a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ServoRead-response)))
  "Returns md5sum for a message object of type 'ServoRead-response"
  "6685c94dd9155802f37ed34fb627a83a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ServoRead-response>)))
  "Returns full string definition for message of type '<ServoRead-response>"
  (cl:format cl:nil "float32 value~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ServoRead-response)))
  "Returns full string definition for message of type 'ServoRead-response"
  (cl:format cl:nil "float32 value~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ServoRead-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ServoRead-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ServoRead-response
    (cl:cons ':value (value msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ServoRead)))
  'ServoRead-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ServoRead)))
  'ServoRead-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ServoRead)))
  "Returns string type for a service object of type '<ServoRead>"
  "ros_arduino_msgs/ServoRead")
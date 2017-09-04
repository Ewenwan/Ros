; Auto-generated. Do not edit!


(cl:in-package industrial_msgs-srv)


;//! \htmlinclude SetRemoteLoggerLevel-request.msg.html

(cl:defclass <SetRemoteLoggerLevel-request> (roslisp-msg-protocol:ros-message)
  ((level
    :reader level
    :initarg :level
    :type industrial_msgs-msg:DebugLevel
    :initform (cl:make-instance 'industrial_msgs-msg:DebugLevel)))
)

(cl:defclass SetRemoteLoggerLevel-request (<SetRemoteLoggerLevel-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetRemoteLoggerLevel-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetRemoteLoggerLevel-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name industrial_msgs-srv:<SetRemoteLoggerLevel-request> is deprecated: use industrial_msgs-srv:SetRemoteLoggerLevel-request instead.")))

(cl:ensure-generic-function 'level-val :lambda-list '(m))
(cl:defmethod level-val ((m <SetRemoteLoggerLevel-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader industrial_msgs-srv:level-val is deprecated.  Use industrial_msgs-srv:level instead.")
  (level m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetRemoteLoggerLevel-request>) ostream)
  "Serializes a message object of type '<SetRemoteLoggerLevel-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'level) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetRemoteLoggerLevel-request>) istream)
  "Deserializes a message object of type '<SetRemoteLoggerLevel-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'level) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetRemoteLoggerLevel-request>)))
  "Returns string type for a service object of type '<SetRemoteLoggerLevel-request>"
  "industrial_msgs/SetRemoteLoggerLevelRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetRemoteLoggerLevel-request)))
  "Returns string type for a service object of type 'SetRemoteLoggerLevel-request"
  "industrial_msgs/SetRemoteLoggerLevelRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetRemoteLoggerLevel-request>)))
  "Returns md5sum for a message object of type '<SetRemoteLoggerLevel-request>"
  "4ae8385cb830c0d46129570c3394af54")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetRemoteLoggerLevel-request)))
  "Returns md5sum for a message object of type 'SetRemoteLoggerLevel-request"
  "4ae8385cb830c0d46129570c3394af54")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetRemoteLoggerLevel-request>)))
  "Returns full string definition for message of type '<SetRemoteLoggerLevel-request>"
  (cl:format cl:nil "~%~%~%~%~%industrial_msgs/DebugLevel level~%~%================================================================================~%MSG: industrial_msgs/DebugLevel~%# Debug level message enumeration.  This may replicate some functionality that~%# alreay exists in the ROS logger.~%# TODO: Get more information on the ROS Logger.~%uint8 val~%~%uint8 DEBUG = 5~%uint8 INFO = 4~%uint8 WARN = 3~%uint8 ERROR = 2~%uint8 FATAL = 1~%uint8 NONE = 0 ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetRemoteLoggerLevel-request)))
  "Returns full string definition for message of type 'SetRemoteLoggerLevel-request"
  (cl:format cl:nil "~%~%~%~%~%industrial_msgs/DebugLevel level~%~%================================================================================~%MSG: industrial_msgs/DebugLevel~%# Debug level message enumeration.  This may replicate some functionality that~%# alreay exists in the ROS logger.~%# TODO: Get more information on the ROS Logger.~%uint8 val~%~%uint8 DEBUG = 5~%uint8 INFO = 4~%uint8 WARN = 3~%uint8 ERROR = 2~%uint8 FATAL = 1~%uint8 NONE = 0 ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetRemoteLoggerLevel-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'level))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetRemoteLoggerLevel-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetRemoteLoggerLevel-request
    (cl:cons ':level (level msg))
))
;//! \htmlinclude SetRemoteLoggerLevel-response.msg.html

(cl:defclass <SetRemoteLoggerLevel-response> (roslisp-msg-protocol:ros-message)
  ((code
    :reader code
    :initarg :code
    :type industrial_msgs-msg:ServiceReturnCode
    :initform (cl:make-instance 'industrial_msgs-msg:ServiceReturnCode)))
)

(cl:defclass SetRemoteLoggerLevel-response (<SetRemoteLoggerLevel-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetRemoteLoggerLevel-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetRemoteLoggerLevel-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name industrial_msgs-srv:<SetRemoteLoggerLevel-response> is deprecated: use industrial_msgs-srv:SetRemoteLoggerLevel-response instead.")))

(cl:ensure-generic-function 'code-val :lambda-list '(m))
(cl:defmethod code-val ((m <SetRemoteLoggerLevel-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader industrial_msgs-srv:code-val is deprecated.  Use industrial_msgs-srv:code instead.")
  (code m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetRemoteLoggerLevel-response>) ostream)
  "Serializes a message object of type '<SetRemoteLoggerLevel-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'code) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetRemoteLoggerLevel-response>) istream)
  "Deserializes a message object of type '<SetRemoteLoggerLevel-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'code) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetRemoteLoggerLevel-response>)))
  "Returns string type for a service object of type '<SetRemoteLoggerLevel-response>"
  "industrial_msgs/SetRemoteLoggerLevelResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetRemoteLoggerLevel-response)))
  "Returns string type for a service object of type 'SetRemoteLoggerLevel-response"
  "industrial_msgs/SetRemoteLoggerLevelResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetRemoteLoggerLevel-response>)))
  "Returns md5sum for a message object of type '<SetRemoteLoggerLevel-response>"
  "4ae8385cb830c0d46129570c3394af54")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetRemoteLoggerLevel-response)))
  "Returns md5sum for a message object of type 'SetRemoteLoggerLevel-response"
  "4ae8385cb830c0d46129570c3394af54")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetRemoteLoggerLevel-response>)))
  "Returns full string definition for message of type '<SetRemoteLoggerLevel-response>"
  (cl:format cl:nil "industrial_msgs/ServiceReturnCode code~%~%~%================================================================================~%MSG: industrial_msgs/ServiceReturnCode~%# Service return codes for simple requests.  All ROS-Industrial service~%# replies are required to have a return code indicating success or failure~%# Specific return codes for different failure should be negative.~%int8 val~%~%int8 SUCCESS = 1~%int8 FAILURE = -1~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetRemoteLoggerLevel-response)))
  "Returns full string definition for message of type 'SetRemoteLoggerLevel-response"
  (cl:format cl:nil "industrial_msgs/ServiceReturnCode code~%~%~%================================================================================~%MSG: industrial_msgs/ServiceReturnCode~%# Service return codes for simple requests.  All ROS-Industrial service~%# replies are required to have a return code indicating success or failure~%# Specific return codes for different failure should be negative.~%int8 val~%~%int8 SUCCESS = 1~%int8 FAILURE = -1~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetRemoteLoggerLevel-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'code))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetRemoteLoggerLevel-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetRemoteLoggerLevel-response
    (cl:cons ':code (code msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetRemoteLoggerLevel)))
  'SetRemoteLoggerLevel-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetRemoteLoggerLevel)))
  'SetRemoteLoggerLevel-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetRemoteLoggerLevel)))
  "Returns string type for a service object of type '<SetRemoteLoggerLevel>"
  "industrial_msgs/SetRemoteLoggerLevel")
; Auto-generated. Do not edit!


(cl:in-package industrial_msgs-msg)


;//! \htmlinclude DebugLevel.msg.html

(cl:defclass <DebugLevel> (roslisp-msg-protocol:ros-message)
  ((val
    :reader val
    :initarg :val
    :type cl:fixnum
    :initform 0))
)

(cl:defclass DebugLevel (<DebugLevel>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DebugLevel>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DebugLevel)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name industrial_msgs-msg:<DebugLevel> is deprecated: use industrial_msgs-msg:DebugLevel instead.")))

(cl:ensure-generic-function 'val-val :lambda-list '(m))
(cl:defmethod val-val ((m <DebugLevel>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader industrial_msgs-msg:val-val is deprecated.  Use industrial_msgs-msg:val instead.")
  (val m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<DebugLevel>)))
    "Constants for message type '<DebugLevel>"
  '((:DEBUG . 5)
    (:INFO . 4)
    (:WARN . 3)
    (:ERROR . 2)
    (:FATAL . 1)
    (:NONE . 0))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'DebugLevel)))
    "Constants for message type 'DebugLevel"
  '((:DEBUG . 5)
    (:INFO . 4)
    (:WARN . 3)
    (:ERROR . 2)
    (:FATAL . 1)
    (:NONE . 0))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DebugLevel>) ostream)
  "Serializes a message object of type '<DebugLevel>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'val)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DebugLevel>) istream)
  "Deserializes a message object of type '<DebugLevel>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'val)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DebugLevel>)))
  "Returns string type for a message object of type '<DebugLevel>"
  "industrial_msgs/DebugLevel")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DebugLevel)))
  "Returns string type for a message object of type 'DebugLevel"
  "industrial_msgs/DebugLevel")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DebugLevel>)))
  "Returns md5sum for a message object of type '<DebugLevel>"
  "5bfde194fd95d83abdb02a03ee48fbe7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DebugLevel)))
  "Returns md5sum for a message object of type 'DebugLevel"
  "5bfde194fd95d83abdb02a03ee48fbe7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DebugLevel>)))
  "Returns full string definition for message of type '<DebugLevel>"
  (cl:format cl:nil "# Debug level message enumeration.  This may replicate some functionality that~%# alreay exists in the ROS logger.~%# TODO: Get more information on the ROS Logger.~%uint8 val~%~%uint8 DEBUG = 5~%uint8 INFO = 4~%uint8 WARN = 3~%uint8 ERROR = 2~%uint8 FATAL = 1~%uint8 NONE = 0 ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DebugLevel)))
  "Returns full string definition for message of type 'DebugLevel"
  (cl:format cl:nil "# Debug level message enumeration.  This may replicate some functionality that~%# alreay exists in the ROS logger.~%# TODO: Get more information on the ROS Logger.~%uint8 val~%~%uint8 DEBUG = 5~%uint8 INFO = 4~%uint8 WARN = 3~%uint8 ERROR = 2~%uint8 FATAL = 1~%uint8 NONE = 0 ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DebugLevel>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DebugLevel>))
  "Converts a ROS message object to a list"
  (cl:list 'DebugLevel
    (cl:cons ':val (val msg))
))

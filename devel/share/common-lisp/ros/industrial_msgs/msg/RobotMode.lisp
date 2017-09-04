; Auto-generated. Do not edit!


(cl:in-package industrial_msgs-msg)


;//! \htmlinclude RobotMode.msg.html

(cl:defclass <RobotMode> (roslisp-msg-protocol:ros-message)
  ((val
    :reader val
    :initarg :val
    :type cl:fixnum
    :initform 0))
)

(cl:defclass RobotMode (<RobotMode>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RobotMode>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RobotMode)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name industrial_msgs-msg:<RobotMode> is deprecated: use industrial_msgs-msg:RobotMode instead.")))

(cl:ensure-generic-function 'val-val :lambda-list '(m))
(cl:defmethod val-val ((m <RobotMode>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader industrial_msgs-msg:val-val is deprecated.  Use industrial_msgs-msg:val instead.")
  (val m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<RobotMode>)))
    "Constants for message type '<RobotMode>"
  '((:UNKNOWN . -1)
    (:MANUAL . 1)
    (:AUTO . 2))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'RobotMode)))
    "Constants for message type 'RobotMode"
  '((:UNKNOWN . -1)
    (:MANUAL . 1)
    (:AUTO . 2))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RobotMode>) ostream)
  "Serializes a message object of type '<RobotMode>"
  (cl:let* ((signed (cl:slot-value msg 'val)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RobotMode>) istream)
  "Deserializes a message object of type '<RobotMode>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'val) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RobotMode>)))
  "Returns string type for a message object of type '<RobotMode>"
  "industrial_msgs/RobotMode")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RobotMode)))
  "Returns string type for a message object of type 'RobotMode"
  "industrial_msgs/RobotMode")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RobotMode>)))
  "Returns md5sum for a message object of type '<RobotMode>"
  "24ac279e988b6b3db836e437e6ed1ba0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RobotMode)))
  "Returns md5sum for a message object of type 'RobotMode"
  "24ac279e988b6b3db836e437e6ed1ba0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RobotMode>)))
  "Returns full string definition for message of type '<RobotMode>"
  (cl:format cl:nil "# The Robot mode message encapsulates the mode/teach state of the robot~%# Typically this is controlled by the pendant key switch, but not always~%~%int8 val~%~%# enumerated values~%int8 UNKNOWN=-1                 # Unknown or unavailable         ~%int8 MANUAL=1 			 # Teach OR manual mode~%int8 AUTO=2                     # Automatic mode~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RobotMode)))
  "Returns full string definition for message of type 'RobotMode"
  (cl:format cl:nil "# The Robot mode message encapsulates the mode/teach state of the robot~%# Typically this is controlled by the pendant key switch, but not always~%~%int8 val~%~%# enumerated values~%int8 UNKNOWN=-1                 # Unknown or unavailable         ~%int8 MANUAL=1 			 # Teach OR manual mode~%int8 AUTO=2                     # Automatic mode~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RobotMode>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RobotMode>))
  "Converts a ROS message object to a list"
  (cl:list 'RobotMode
    (cl:cons ':val (val msg))
))

; Auto-generated. Do not edit!


(cl:in-package industrial_msgs-msg)


;//! \htmlinclude TriState.msg.html

(cl:defclass <TriState> (roslisp-msg-protocol:ros-message)
  ((val
    :reader val
    :initarg :val
    :type cl:fixnum
    :initform 0))
)

(cl:defclass TriState (<TriState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TriState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TriState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name industrial_msgs-msg:<TriState> is deprecated: use industrial_msgs-msg:TriState instead.")))

(cl:ensure-generic-function 'val-val :lambda-list '(m))
(cl:defmethod val-val ((m <TriState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader industrial_msgs-msg:val-val is deprecated.  Use industrial_msgs-msg:val instead.")
  (val m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<TriState>)))
    "Constants for message type '<TriState>"
  '((:UNKNOWN . -1)
    (:TRUE . 1)
    (:ON . 1)
    (:ENABLED . 1)
    (:HIGH . 1)
    (:CLOSED . 1)
    (:FALSE . 0)
    (:OFF . 0)
    (:DISABLED . 0)
    (:LOW . 0)
    (:OPEN . 0))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'TriState)))
    "Constants for message type 'TriState"
  '((:UNKNOWN . -1)
    (:TRUE . 1)
    (:ON . 1)
    (:ENABLED . 1)
    (:HIGH . 1)
    (:CLOSED . 1)
    (:FALSE . 0)
    (:OFF . 0)
    (:DISABLED . 0)
    (:LOW . 0)
    (:OPEN . 0))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TriState>) ostream)
  "Serializes a message object of type '<TriState>"
  (cl:let* ((signed (cl:slot-value msg 'val)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TriState>) istream)
  "Deserializes a message object of type '<TriState>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'val) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TriState>)))
  "Returns string type for a message object of type '<TriState>"
  "industrial_msgs/TriState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TriState)))
  "Returns string type for a message object of type 'TriState"
  "industrial_msgs/TriState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TriState>)))
  "Returns md5sum for a message object of type '<TriState>"
  "deb03829f3c2d0f1b647fa38d7087952")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TriState)))
  "Returns md5sum for a message object of type 'TriState"
  "deb03829f3c2d0f1b647fa38d7087952")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TriState>)))
  "Returns full string definition for message of type '<TriState>"
  (cl:format cl:nil "# The tri-state captures boolean values with the additional state of unknown~%~%int8 val~%~%# enumerated values~%~%# Unknown or unavailable ~%int8 UNKNOWN=-1  ~%~%# High state                       ~%int8 TRUE=1~%int8 ON=1~%int8 ENABLED=1~%int8 HIGH=1~%int8 CLOSED=1~%~%# Low state~%int8 FALSE=0~%int8 OFF=0~%int8 DISABLED=0~%int8 LOW=0~%int8 OPEN=0~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TriState)))
  "Returns full string definition for message of type 'TriState"
  (cl:format cl:nil "# The tri-state captures boolean values with the additional state of unknown~%~%int8 val~%~%# enumerated values~%~%# Unknown or unavailable ~%int8 UNKNOWN=-1  ~%~%# High state                       ~%int8 TRUE=1~%int8 ON=1~%int8 ENABLED=1~%int8 HIGH=1~%int8 CLOSED=1~%~%# Low state~%int8 FALSE=0~%int8 OFF=0~%int8 DISABLED=0~%int8 LOW=0~%int8 OPEN=0~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TriState>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TriState>))
  "Converts a ROS message object to a list"
  (cl:list 'TriState
    (cl:cons ':val (val msg))
))

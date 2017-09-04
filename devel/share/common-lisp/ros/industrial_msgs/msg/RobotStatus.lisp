; Auto-generated. Do not edit!


(cl:in-package industrial_msgs-msg)


;//! \htmlinclude RobotStatus.msg.html

(cl:defclass <RobotStatus> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (mode
    :reader mode
    :initarg :mode
    :type industrial_msgs-msg:RobotMode
    :initform (cl:make-instance 'industrial_msgs-msg:RobotMode))
   (e_stopped
    :reader e_stopped
    :initarg :e_stopped
    :type industrial_msgs-msg:TriState
    :initform (cl:make-instance 'industrial_msgs-msg:TriState))
   (drives_powered
    :reader drives_powered
    :initarg :drives_powered
    :type industrial_msgs-msg:TriState
    :initform (cl:make-instance 'industrial_msgs-msg:TriState))
   (motion_possible
    :reader motion_possible
    :initarg :motion_possible
    :type industrial_msgs-msg:TriState
    :initform (cl:make-instance 'industrial_msgs-msg:TriState))
   (in_motion
    :reader in_motion
    :initarg :in_motion
    :type industrial_msgs-msg:TriState
    :initform (cl:make-instance 'industrial_msgs-msg:TriState))
   (in_error
    :reader in_error
    :initarg :in_error
    :type industrial_msgs-msg:TriState
    :initform (cl:make-instance 'industrial_msgs-msg:TriState))
   (error_code
    :reader error_code
    :initarg :error_code
    :type cl:integer
    :initform 0))
)

(cl:defclass RobotStatus (<RobotStatus>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RobotStatus>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RobotStatus)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name industrial_msgs-msg:<RobotStatus> is deprecated: use industrial_msgs-msg:RobotStatus instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <RobotStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader industrial_msgs-msg:header-val is deprecated.  Use industrial_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <RobotStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader industrial_msgs-msg:mode-val is deprecated.  Use industrial_msgs-msg:mode instead.")
  (mode m))

(cl:ensure-generic-function 'e_stopped-val :lambda-list '(m))
(cl:defmethod e_stopped-val ((m <RobotStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader industrial_msgs-msg:e_stopped-val is deprecated.  Use industrial_msgs-msg:e_stopped instead.")
  (e_stopped m))

(cl:ensure-generic-function 'drives_powered-val :lambda-list '(m))
(cl:defmethod drives_powered-val ((m <RobotStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader industrial_msgs-msg:drives_powered-val is deprecated.  Use industrial_msgs-msg:drives_powered instead.")
  (drives_powered m))

(cl:ensure-generic-function 'motion_possible-val :lambda-list '(m))
(cl:defmethod motion_possible-val ((m <RobotStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader industrial_msgs-msg:motion_possible-val is deprecated.  Use industrial_msgs-msg:motion_possible instead.")
  (motion_possible m))

(cl:ensure-generic-function 'in_motion-val :lambda-list '(m))
(cl:defmethod in_motion-val ((m <RobotStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader industrial_msgs-msg:in_motion-val is deprecated.  Use industrial_msgs-msg:in_motion instead.")
  (in_motion m))

(cl:ensure-generic-function 'in_error-val :lambda-list '(m))
(cl:defmethod in_error-val ((m <RobotStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader industrial_msgs-msg:in_error-val is deprecated.  Use industrial_msgs-msg:in_error instead.")
  (in_error m))

(cl:ensure-generic-function 'error_code-val :lambda-list '(m))
(cl:defmethod error_code-val ((m <RobotStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader industrial_msgs-msg:error_code-val is deprecated.  Use industrial_msgs-msg:error_code instead.")
  (error_code m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RobotStatus>) ostream)
  "Serializes a message object of type '<RobotStatus>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'mode) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'e_stopped) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'drives_powered) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'motion_possible) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'in_motion) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'in_error) ostream)
  (cl:let* ((signed (cl:slot-value msg 'error_code)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RobotStatus>) istream)
  "Deserializes a message object of type '<RobotStatus>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'mode) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'e_stopped) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'drives_powered) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'motion_possible) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'in_motion) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'in_error) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'error_code) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RobotStatus>)))
  "Returns string type for a message object of type '<RobotStatus>"
  "industrial_msgs/RobotStatus")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RobotStatus)))
  "Returns string type for a message object of type 'RobotStatus"
  "industrial_msgs/RobotStatus")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RobotStatus>)))
  "Returns md5sum for a message object of type '<RobotStatus>"
  "b733cb45a38101840b75c4c0d6093d19")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RobotStatus)))
  "Returns md5sum for a message object of type 'RobotStatus"
  "b733cb45a38101840b75c4c0d6093d19")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RobotStatus>)))
  "Returns full string definition for message of type '<RobotStatus>"
  (cl:format cl:nil "# The RobotStatus message contains low level status information ~%# that is specific to an industrial robot controller~%~%# The header frame ID is not used~%Header header~%~%# The robot mode captures the operating mode of the robot.  When in~%# manual, remote motion is not possible.~%industrial_msgs/RobotMode mode~%~%# Estop status: True if robot is e-stopped.  The drives are disabled~%# and motion is not possible.  The e-stop condition must be acknowledged~%# and cleared before any motion can begin.~%industrial_msgs/TriState e_stopped~%~%# Drive power status: True if drives are powered.  Motion commands will ~%# automatically enable the drives if required.  Drive power is not requred~%# for possible motion~%industrial_msgs/TriState drives_powered~%~%# Motion enabled: Ture if robot motion is possible.~%industrial_msgs/TriState motion_possible~%~%# Motion status: True if robot is in motion, otherwise false~%industrial_msgs/TriState in_motion~%~%# Error status: True if there is an error condition on the robot. Motion may~%# or may not be affected (see motion_possible)~%industrial_msgs/TriState in_error~%~%# Error code: Vendor specific error code (non zero indicates error)~%int32 error_code~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: industrial_msgs/RobotMode~%# The Robot mode message encapsulates the mode/teach state of the robot~%# Typically this is controlled by the pendant key switch, but not always~%~%int8 val~%~%# enumerated values~%int8 UNKNOWN=-1                 # Unknown or unavailable         ~%int8 MANUAL=1 			 # Teach OR manual mode~%int8 AUTO=2                     # Automatic mode~%~%~%================================================================================~%MSG: industrial_msgs/TriState~%# The tri-state captures boolean values with the additional state of unknown~%~%int8 val~%~%# enumerated values~%~%# Unknown or unavailable ~%int8 UNKNOWN=-1  ~%~%# High state                       ~%int8 TRUE=1~%int8 ON=1~%int8 ENABLED=1~%int8 HIGH=1~%int8 CLOSED=1~%~%# Low state~%int8 FALSE=0~%int8 OFF=0~%int8 DISABLED=0~%int8 LOW=0~%int8 OPEN=0~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RobotStatus)))
  "Returns full string definition for message of type 'RobotStatus"
  (cl:format cl:nil "# The RobotStatus message contains low level status information ~%# that is specific to an industrial robot controller~%~%# The header frame ID is not used~%Header header~%~%# The robot mode captures the operating mode of the robot.  When in~%# manual, remote motion is not possible.~%industrial_msgs/RobotMode mode~%~%# Estop status: True if robot is e-stopped.  The drives are disabled~%# and motion is not possible.  The e-stop condition must be acknowledged~%# and cleared before any motion can begin.~%industrial_msgs/TriState e_stopped~%~%# Drive power status: True if drives are powered.  Motion commands will ~%# automatically enable the drives if required.  Drive power is not requred~%# for possible motion~%industrial_msgs/TriState drives_powered~%~%# Motion enabled: Ture if robot motion is possible.~%industrial_msgs/TriState motion_possible~%~%# Motion status: True if robot is in motion, otherwise false~%industrial_msgs/TriState in_motion~%~%# Error status: True if there is an error condition on the robot. Motion may~%# or may not be affected (see motion_possible)~%industrial_msgs/TriState in_error~%~%# Error code: Vendor specific error code (non zero indicates error)~%int32 error_code~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: industrial_msgs/RobotMode~%# The Robot mode message encapsulates the mode/teach state of the robot~%# Typically this is controlled by the pendant key switch, but not always~%~%int8 val~%~%# enumerated values~%int8 UNKNOWN=-1                 # Unknown or unavailable         ~%int8 MANUAL=1 			 # Teach OR manual mode~%int8 AUTO=2                     # Automatic mode~%~%~%================================================================================~%MSG: industrial_msgs/TriState~%# The tri-state captures boolean values with the additional state of unknown~%~%int8 val~%~%# enumerated values~%~%# Unknown or unavailable ~%int8 UNKNOWN=-1  ~%~%# High state                       ~%int8 TRUE=1~%int8 ON=1~%int8 ENABLED=1~%int8 HIGH=1~%int8 CLOSED=1~%~%# Low state~%int8 FALSE=0~%int8 OFF=0~%int8 DISABLED=0~%int8 LOW=0~%int8 OPEN=0~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RobotStatus>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'mode))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'e_stopped))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'drives_powered))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'motion_possible))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'in_motion))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'in_error))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RobotStatus>))
  "Converts a ROS message object to a list"
  (cl:list 'RobotStatus
    (cl:cons ':header (header msg))
    (cl:cons ':mode (mode msg))
    (cl:cons ':e_stopped (e_stopped msg))
    (cl:cons ':drives_powered (drives_powered msg))
    (cl:cons ':motion_possible (motion_possible msg))
    (cl:cons ':in_motion (in_motion msg))
    (cl:cons ':in_error (in_error msg))
    (cl:cons ':error_code (error_code msg))
))

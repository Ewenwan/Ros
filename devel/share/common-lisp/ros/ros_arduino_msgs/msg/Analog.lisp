; Auto-generated. Do not edit!


(cl:in-package ros_arduino_msgs-msg)


;//! \htmlinclude Analog.msg.html

(cl:defclass <Analog> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (value
    :reader value
    :initarg :value
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Analog (<Analog>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Analog>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Analog)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ros_arduino_msgs-msg:<Analog> is deprecated: use ros_arduino_msgs-msg:Analog instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Analog>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_arduino_msgs-msg:header-val is deprecated.  Use ros_arduino_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <Analog>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_arduino_msgs-msg:value-val is deprecated.  Use ros_arduino_msgs-msg:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Analog>) ostream)
  "Serializes a message object of type '<Analog>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'value)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'value)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Analog>) istream)
  "Deserializes a message object of type '<Analog>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'value)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'value)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Analog>)))
  "Returns string type for a message object of type '<Analog>"
  "ros_arduino_msgs/Analog")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Analog)))
  "Returns string type for a message object of type 'Analog"
  "ros_arduino_msgs/Analog")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Analog>)))
  "Returns md5sum for a message object of type '<Analog>"
  "5760aa9c40c2caa52a04d293094e679d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Analog)))
  "Returns md5sum for a message object of type 'Analog"
  "5760aa9c40c2caa52a04d293094e679d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Analog>)))
  "Returns full string definition for message of type '<Analog>"
  (cl:format cl:nil "# Reading from a single analog IO pin.~%Header header~%uint16 value~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Analog)))
  "Returns full string definition for message of type 'Analog"
  (cl:format cl:nil "# Reading from a single analog IO pin.~%Header header~%uint16 value~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Analog>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Analog>))
  "Converts a ROS message object to a list"
  (cl:list 'Analog
    (cl:cons ':header (header msg))
    (cl:cons ':value (value msg))
))

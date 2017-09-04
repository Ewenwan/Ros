; Auto-generated. Do not edit!


(cl:in-package industrial_msgs-srv)


;//! \htmlinclude CmdJointTrajectory-request.msg.html

(cl:defclass <CmdJointTrajectory-request> (roslisp-msg-protocol:ros-message)
  ((trajectory
    :reader trajectory
    :initarg :trajectory
    :type trajectory_msgs-msg:JointTrajectory
    :initform (cl:make-instance 'trajectory_msgs-msg:JointTrajectory)))
)

(cl:defclass CmdJointTrajectory-request (<CmdJointTrajectory-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CmdJointTrajectory-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CmdJointTrajectory-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name industrial_msgs-srv:<CmdJointTrajectory-request> is deprecated: use industrial_msgs-srv:CmdJointTrajectory-request instead.")))

(cl:ensure-generic-function 'trajectory-val :lambda-list '(m))
(cl:defmethod trajectory-val ((m <CmdJointTrajectory-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader industrial_msgs-srv:trajectory-val is deprecated.  Use industrial_msgs-srv:trajectory instead.")
  (trajectory m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CmdJointTrajectory-request>) ostream)
  "Serializes a message object of type '<CmdJointTrajectory-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'trajectory) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CmdJointTrajectory-request>) istream)
  "Deserializes a message object of type '<CmdJointTrajectory-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'trajectory) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CmdJointTrajectory-request>)))
  "Returns string type for a service object of type '<CmdJointTrajectory-request>"
  "industrial_msgs/CmdJointTrajectoryRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CmdJointTrajectory-request)))
  "Returns string type for a service object of type 'CmdJointTrajectory-request"
  "industrial_msgs/CmdJointTrajectoryRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CmdJointTrajectory-request>)))
  "Returns md5sum for a message object of type '<CmdJointTrajectory-request>"
  "94fdf82abbbb1071bc31be1a2aea4fcd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CmdJointTrajectory-request)))
  "Returns md5sum for a message object of type 'CmdJointTrajectory-request"
  "94fdf82abbbb1071bc31be1a2aea4fcd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CmdJointTrajectory-request>)))
  "Returns full string definition for message of type '<CmdJointTrajectory-request>"
  (cl:format cl:nil "~%~%~%~%~%~%trajectory_msgs/JointTrajectory trajectory~%~%================================================================================~%MSG: trajectory_msgs/JointTrajectory~%Header header~%string[] joint_names~%JointTrajectoryPoint[] points~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: trajectory_msgs/JointTrajectoryPoint~%# Each trajectory point specifies either positions[, velocities[, accelerations]]~%# or positions[, effort] for the trajectory to be executed.~%# All specified values are in the same order as the joint names in JointTrajectory.msg~%~%float64[] positions~%float64[] velocities~%float64[] accelerations~%float64[] effort~%duration time_from_start~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CmdJointTrajectory-request)))
  "Returns full string definition for message of type 'CmdJointTrajectory-request"
  (cl:format cl:nil "~%~%~%~%~%~%trajectory_msgs/JointTrajectory trajectory~%~%================================================================================~%MSG: trajectory_msgs/JointTrajectory~%Header header~%string[] joint_names~%JointTrajectoryPoint[] points~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: trajectory_msgs/JointTrajectoryPoint~%# Each trajectory point specifies either positions[, velocities[, accelerations]]~%# or positions[, effort] for the trajectory to be executed.~%# All specified values are in the same order as the joint names in JointTrajectory.msg~%~%float64[] positions~%float64[] velocities~%float64[] accelerations~%float64[] effort~%duration time_from_start~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CmdJointTrajectory-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'trajectory))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CmdJointTrajectory-request>))
  "Converts a ROS message object to a list"
  (cl:list 'CmdJointTrajectory-request
    (cl:cons ':trajectory (trajectory msg))
))
;//! \htmlinclude CmdJointTrajectory-response.msg.html

(cl:defclass <CmdJointTrajectory-response> (roslisp-msg-protocol:ros-message)
  ((code
    :reader code
    :initarg :code
    :type industrial_msgs-msg:ServiceReturnCode
    :initform (cl:make-instance 'industrial_msgs-msg:ServiceReturnCode)))
)

(cl:defclass CmdJointTrajectory-response (<CmdJointTrajectory-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CmdJointTrajectory-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CmdJointTrajectory-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name industrial_msgs-srv:<CmdJointTrajectory-response> is deprecated: use industrial_msgs-srv:CmdJointTrajectory-response instead.")))

(cl:ensure-generic-function 'code-val :lambda-list '(m))
(cl:defmethod code-val ((m <CmdJointTrajectory-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader industrial_msgs-srv:code-val is deprecated.  Use industrial_msgs-srv:code instead.")
  (code m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CmdJointTrajectory-response>) ostream)
  "Serializes a message object of type '<CmdJointTrajectory-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'code) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CmdJointTrajectory-response>) istream)
  "Deserializes a message object of type '<CmdJointTrajectory-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'code) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CmdJointTrajectory-response>)))
  "Returns string type for a service object of type '<CmdJointTrajectory-response>"
  "industrial_msgs/CmdJointTrajectoryResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CmdJointTrajectory-response)))
  "Returns string type for a service object of type 'CmdJointTrajectory-response"
  "industrial_msgs/CmdJointTrajectoryResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CmdJointTrajectory-response>)))
  "Returns md5sum for a message object of type '<CmdJointTrajectory-response>"
  "94fdf82abbbb1071bc31be1a2aea4fcd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CmdJointTrajectory-response)))
  "Returns md5sum for a message object of type 'CmdJointTrajectory-response"
  "94fdf82abbbb1071bc31be1a2aea4fcd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CmdJointTrajectory-response>)))
  "Returns full string definition for message of type '<CmdJointTrajectory-response>"
  (cl:format cl:nil "industrial_msgs/ServiceReturnCode code~%~%~%================================================================================~%MSG: industrial_msgs/ServiceReturnCode~%# Service return codes for simple requests.  All ROS-Industrial service~%# replies are required to have a return code indicating success or failure~%# Specific return codes for different failure should be negative.~%int8 val~%~%int8 SUCCESS = 1~%int8 FAILURE = -1~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CmdJointTrajectory-response)))
  "Returns full string definition for message of type 'CmdJointTrajectory-response"
  (cl:format cl:nil "industrial_msgs/ServiceReturnCode code~%~%~%================================================================================~%MSG: industrial_msgs/ServiceReturnCode~%# Service return codes for simple requests.  All ROS-Industrial service~%# replies are required to have a return code indicating success or failure~%# Specific return codes for different failure should be negative.~%int8 val~%~%int8 SUCCESS = 1~%int8 FAILURE = -1~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CmdJointTrajectory-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'code))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CmdJointTrajectory-response>))
  "Converts a ROS message object to a list"
  (cl:list 'CmdJointTrajectory-response
    (cl:cons ':code (code msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'CmdJointTrajectory)))
  'CmdJointTrajectory-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'CmdJointTrajectory)))
  'CmdJointTrajectory-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CmdJointTrajectory)))
  "Returns string type for a service object of type '<CmdJointTrajectory>"
  "industrial_msgs/CmdJointTrajectory")
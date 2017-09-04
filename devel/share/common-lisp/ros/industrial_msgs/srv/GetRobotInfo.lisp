; Auto-generated. Do not edit!


(cl:in-package industrial_msgs-srv)


;//! \htmlinclude GetRobotInfo-request.msg.html

(cl:defclass <GetRobotInfo-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetRobotInfo-request (<GetRobotInfo-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetRobotInfo-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetRobotInfo-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name industrial_msgs-srv:<GetRobotInfo-request> is deprecated: use industrial_msgs-srv:GetRobotInfo-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetRobotInfo-request>) ostream)
  "Serializes a message object of type '<GetRobotInfo-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetRobotInfo-request>) istream)
  "Deserializes a message object of type '<GetRobotInfo-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetRobotInfo-request>)))
  "Returns string type for a service object of type '<GetRobotInfo-request>"
  "industrial_msgs/GetRobotInfoRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetRobotInfo-request)))
  "Returns string type for a service object of type 'GetRobotInfo-request"
  "industrial_msgs/GetRobotInfoRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetRobotInfo-request>)))
  "Returns md5sum for a message object of type '<GetRobotInfo-request>"
  "5db3230b3e61c85a320b999ffd7f3b3f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetRobotInfo-request)))
  "Returns md5sum for a message object of type 'GetRobotInfo-request"
  "5db3230b3e61c85a320b999ffd7f3b3f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetRobotInfo-request>)))
  "Returns full string definition for message of type '<GetRobotInfo-request>"
  (cl:format cl:nil "~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetRobotInfo-request)))
  "Returns full string definition for message of type 'GetRobotInfo-request"
  (cl:format cl:nil "~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetRobotInfo-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetRobotInfo-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetRobotInfo-request
))
;//! \htmlinclude GetRobotInfo-response.msg.html

(cl:defclass <GetRobotInfo-response> (roslisp-msg-protocol:ros-message)
  ((controller
    :reader controller
    :initarg :controller
    :type industrial_msgs-msg:DeviceInfo
    :initform (cl:make-instance 'industrial_msgs-msg:DeviceInfo))
   (robots
    :reader robots
    :initarg :robots
    :type (cl:vector industrial_msgs-msg:DeviceInfo)
   :initform (cl:make-array 0 :element-type 'industrial_msgs-msg:DeviceInfo :initial-element (cl:make-instance 'industrial_msgs-msg:DeviceInfo)))
   (code
    :reader code
    :initarg :code
    :type industrial_msgs-msg:ServiceReturnCode
    :initform (cl:make-instance 'industrial_msgs-msg:ServiceReturnCode)))
)

(cl:defclass GetRobotInfo-response (<GetRobotInfo-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetRobotInfo-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetRobotInfo-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name industrial_msgs-srv:<GetRobotInfo-response> is deprecated: use industrial_msgs-srv:GetRobotInfo-response instead.")))

(cl:ensure-generic-function 'controller-val :lambda-list '(m))
(cl:defmethod controller-val ((m <GetRobotInfo-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader industrial_msgs-srv:controller-val is deprecated.  Use industrial_msgs-srv:controller instead.")
  (controller m))

(cl:ensure-generic-function 'robots-val :lambda-list '(m))
(cl:defmethod robots-val ((m <GetRobotInfo-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader industrial_msgs-srv:robots-val is deprecated.  Use industrial_msgs-srv:robots instead.")
  (robots m))

(cl:ensure-generic-function 'code-val :lambda-list '(m))
(cl:defmethod code-val ((m <GetRobotInfo-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader industrial_msgs-srv:code-val is deprecated.  Use industrial_msgs-srv:code instead.")
  (code m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetRobotInfo-response>) ostream)
  "Serializes a message object of type '<GetRobotInfo-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'controller) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'robots))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'robots))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'code) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetRobotInfo-response>) istream)
  "Deserializes a message object of type '<GetRobotInfo-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'controller) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'robots) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'robots)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'industrial_msgs-msg:DeviceInfo))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'code) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetRobotInfo-response>)))
  "Returns string type for a service object of type '<GetRobotInfo-response>"
  "industrial_msgs/GetRobotInfoResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetRobotInfo-response)))
  "Returns string type for a service object of type 'GetRobotInfo-response"
  "industrial_msgs/GetRobotInfoResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetRobotInfo-response>)))
  "Returns md5sum for a message object of type '<GetRobotInfo-response>"
  "5db3230b3e61c85a320b999ffd7f3b3f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetRobotInfo-response)))
  "Returns md5sum for a message object of type 'GetRobotInfo-response"
  "5db3230b3e61c85a320b999ffd7f3b3f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetRobotInfo-response>)))
  "Returns full string definition for message of type '<GetRobotInfo-response>"
  (cl:format cl:nil "industrial_msgs/DeviceInfo controller~%industrial_msgs/DeviceInfo[] robots~%industrial_msgs/ServiceReturnCode code~%~%~%================================================================================~%MSG: industrial_msgs/DeviceInfo~%# Device info captures device agnostic information about a piece of hardware.~%# This message is meant as a generic as possible.  Items that don't apply should~%# be left blank.  This message is not meant to replace diagnostic messages, but~%# rather provide a standard service message that can be used to populate standard~%# components (like a GUI for example)~%~%string model~%string serial_number~%string hw_version~%string sw_version~%string address~%~%================================================================================~%MSG: industrial_msgs/ServiceReturnCode~%# Service return codes for simple requests.  All ROS-Industrial service~%# replies are required to have a return code indicating success or failure~%# Specific return codes for different failure should be negative.~%int8 val~%~%int8 SUCCESS = 1~%int8 FAILURE = -1~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetRobotInfo-response)))
  "Returns full string definition for message of type 'GetRobotInfo-response"
  (cl:format cl:nil "industrial_msgs/DeviceInfo controller~%industrial_msgs/DeviceInfo[] robots~%industrial_msgs/ServiceReturnCode code~%~%~%================================================================================~%MSG: industrial_msgs/DeviceInfo~%# Device info captures device agnostic information about a piece of hardware.~%# This message is meant as a generic as possible.  Items that don't apply should~%# be left blank.  This message is not meant to replace diagnostic messages, but~%# rather provide a standard service message that can be used to populate standard~%# components (like a GUI for example)~%~%string model~%string serial_number~%string hw_version~%string sw_version~%string address~%~%================================================================================~%MSG: industrial_msgs/ServiceReturnCode~%# Service return codes for simple requests.  All ROS-Industrial service~%# replies are required to have a return code indicating success or failure~%# Specific return codes for different failure should be negative.~%int8 val~%~%int8 SUCCESS = 1~%int8 FAILURE = -1~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetRobotInfo-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'controller))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'robots) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'code))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetRobotInfo-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetRobotInfo-response
    (cl:cons ':controller (controller msg))
    (cl:cons ':robots (robots msg))
    (cl:cons ':code (code msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetRobotInfo)))
  'GetRobotInfo-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetRobotInfo)))
  'GetRobotInfo-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetRobotInfo)))
  "Returns string type for a service object of type '<GetRobotInfo>"
  "industrial_msgs/GetRobotInfo")
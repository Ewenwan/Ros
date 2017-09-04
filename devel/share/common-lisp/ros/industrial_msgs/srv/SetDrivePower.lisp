; Auto-generated. Do not edit!


(cl:in-package industrial_msgs-srv)


;//! \htmlinclude SetDrivePower-request.msg.html

(cl:defclass <SetDrivePower-request> (roslisp-msg-protocol:ros-message)
  ((drive_power
    :reader drive_power
    :initarg :drive_power
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetDrivePower-request (<SetDrivePower-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetDrivePower-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetDrivePower-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name industrial_msgs-srv:<SetDrivePower-request> is deprecated: use industrial_msgs-srv:SetDrivePower-request instead.")))

(cl:ensure-generic-function 'drive_power-val :lambda-list '(m))
(cl:defmethod drive_power-val ((m <SetDrivePower-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader industrial_msgs-srv:drive_power-val is deprecated.  Use industrial_msgs-srv:drive_power instead.")
  (drive_power m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetDrivePower-request>) ostream)
  "Serializes a message object of type '<SetDrivePower-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'drive_power) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetDrivePower-request>) istream)
  "Deserializes a message object of type '<SetDrivePower-request>"
    (cl:setf (cl:slot-value msg 'drive_power) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetDrivePower-request>)))
  "Returns string type for a service object of type '<SetDrivePower-request>"
  "industrial_msgs/SetDrivePowerRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetDrivePower-request)))
  "Returns string type for a service object of type 'SetDrivePower-request"
  "industrial_msgs/SetDrivePowerRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetDrivePower-request>)))
  "Returns md5sum for a message object of type '<SetDrivePower-request>"
  "89dc29b38aedf0d168daa33da97de48a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetDrivePower-request)))
  "Returns md5sum for a message object of type 'SetDrivePower-request"
  "89dc29b38aedf0d168daa33da97de48a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetDrivePower-request>)))
  "Returns full string definition for message of type '<SetDrivePower-request>"
  (cl:format cl:nil "~%~%~%~%bool drive_power~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetDrivePower-request)))
  "Returns full string definition for message of type 'SetDrivePower-request"
  (cl:format cl:nil "~%~%~%~%bool drive_power~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetDrivePower-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetDrivePower-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetDrivePower-request
    (cl:cons ':drive_power (drive_power msg))
))
;//! \htmlinclude SetDrivePower-response.msg.html

(cl:defclass <SetDrivePower-response> (roslisp-msg-protocol:ros-message)
  ((code
    :reader code
    :initarg :code
    :type industrial_msgs-msg:ServiceReturnCode
    :initform (cl:make-instance 'industrial_msgs-msg:ServiceReturnCode)))
)

(cl:defclass SetDrivePower-response (<SetDrivePower-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetDrivePower-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetDrivePower-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name industrial_msgs-srv:<SetDrivePower-response> is deprecated: use industrial_msgs-srv:SetDrivePower-response instead.")))

(cl:ensure-generic-function 'code-val :lambda-list '(m))
(cl:defmethod code-val ((m <SetDrivePower-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader industrial_msgs-srv:code-val is deprecated.  Use industrial_msgs-srv:code instead.")
  (code m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetDrivePower-response>) ostream)
  "Serializes a message object of type '<SetDrivePower-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'code) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetDrivePower-response>) istream)
  "Deserializes a message object of type '<SetDrivePower-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'code) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetDrivePower-response>)))
  "Returns string type for a service object of type '<SetDrivePower-response>"
  "industrial_msgs/SetDrivePowerResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetDrivePower-response)))
  "Returns string type for a service object of type 'SetDrivePower-response"
  "industrial_msgs/SetDrivePowerResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetDrivePower-response>)))
  "Returns md5sum for a message object of type '<SetDrivePower-response>"
  "89dc29b38aedf0d168daa33da97de48a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetDrivePower-response)))
  "Returns md5sum for a message object of type 'SetDrivePower-response"
  "89dc29b38aedf0d168daa33da97de48a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetDrivePower-response>)))
  "Returns full string definition for message of type '<SetDrivePower-response>"
  (cl:format cl:nil "industrial_msgs/ServiceReturnCode code~%~%~%================================================================================~%MSG: industrial_msgs/ServiceReturnCode~%# Service return codes for simple requests.  All ROS-Industrial service~%# replies are required to have a return code indicating success or failure~%# Specific return codes for different failure should be negative.~%int8 val~%~%int8 SUCCESS = 1~%int8 FAILURE = -1~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetDrivePower-response)))
  "Returns full string definition for message of type 'SetDrivePower-response"
  (cl:format cl:nil "industrial_msgs/ServiceReturnCode code~%~%~%================================================================================~%MSG: industrial_msgs/ServiceReturnCode~%# Service return codes for simple requests.  All ROS-Industrial service~%# replies are required to have a return code indicating success or failure~%# Specific return codes for different failure should be negative.~%int8 val~%~%int8 SUCCESS = 1~%int8 FAILURE = -1~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetDrivePower-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'code))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetDrivePower-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetDrivePower-response
    (cl:cons ':code (code msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetDrivePower)))
  'SetDrivePower-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetDrivePower)))
  'SetDrivePower-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetDrivePower)))
  "Returns string type for a service object of type '<SetDrivePower>"
  "industrial_msgs/SetDrivePower")
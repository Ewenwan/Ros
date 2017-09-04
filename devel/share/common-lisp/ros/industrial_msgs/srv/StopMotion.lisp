; Auto-generated. Do not edit!


(cl:in-package industrial_msgs-srv)


;//! \htmlinclude StopMotion-request.msg.html

(cl:defclass <StopMotion-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass StopMotion-request (<StopMotion-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StopMotion-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StopMotion-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name industrial_msgs-srv:<StopMotion-request> is deprecated: use industrial_msgs-srv:StopMotion-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StopMotion-request>) ostream)
  "Serializes a message object of type '<StopMotion-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StopMotion-request>) istream)
  "Deserializes a message object of type '<StopMotion-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StopMotion-request>)))
  "Returns string type for a service object of type '<StopMotion-request>"
  "industrial_msgs/StopMotionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StopMotion-request)))
  "Returns string type for a service object of type 'StopMotion-request"
  "industrial_msgs/StopMotionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StopMotion-request>)))
  "Returns md5sum for a message object of type '<StopMotion-request>"
  "50b1f38f75f5677e5692f3b3e7e1ea48")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StopMotion-request)))
  "Returns md5sum for a message object of type 'StopMotion-request"
  "50b1f38f75f5677e5692f3b3e7e1ea48")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StopMotion-request>)))
  "Returns full string definition for message of type '<StopMotion-request>"
  (cl:format cl:nil "~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StopMotion-request)))
  "Returns full string definition for message of type 'StopMotion-request"
  (cl:format cl:nil "~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StopMotion-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StopMotion-request>))
  "Converts a ROS message object to a list"
  (cl:list 'StopMotion-request
))
;//! \htmlinclude StopMotion-response.msg.html

(cl:defclass <StopMotion-response> (roslisp-msg-protocol:ros-message)
  ((code
    :reader code
    :initarg :code
    :type industrial_msgs-msg:ServiceReturnCode
    :initform (cl:make-instance 'industrial_msgs-msg:ServiceReturnCode)))
)

(cl:defclass StopMotion-response (<StopMotion-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StopMotion-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StopMotion-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name industrial_msgs-srv:<StopMotion-response> is deprecated: use industrial_msgs-srv:StopMotion-response instead.")))

(cl:ensure-generic-function 'code-val :lambda-list '(m))
(cl:defmethod code-val ((m <StopMotion-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader industrial_msgs-srv:code-val is deprecated.  Use industrial_msgs-srv:code instead.")
  (code m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StopMotion-response>) ostream)
  "Serializes a message object of type '<StopMotion-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'code) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StopMotion-response>) istream)
  "Deserializes a message object of type '<StopMotion-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'code) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StopMotion-response>)))
  "Returns string type for a service object of type '<StopMotion-response>"
  "industrial_msgs/StopMotionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StopMotion-response)))
  "Returns string type for a service object of type 'StopMotion-response"
  "industrial_msgs/StopMotionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StopMotion-response>)))
  "Returns md5sum for a message object of type '<StopMotion-response>"
  "50b1f38f75f5677e5692f3b3e7e1ea48")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StopMotion-response)))
  "Returns md5sum for a message object of type 'StopMotion-response"
  "50b1f38f75f5677e5692f3b3e7e1ea48")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StopMotion-response>)))
  "Returns full string definition for message of type '<StopMotion-response>"
  (cl:format cl:nil "industrial_msgs/ServiceReturnCode code~%~%~%================================================================================~%MSG: industrial_msgs/ServiceReturnCode~%# Service return codes for simple requests.  All ROS-Industrial service~%# replies are required to have a return code indicating success or failure~%# Specific return codes for different failure should be negative.~%int8 val~%~%int8 SUCCESS = 1~%int8 FAILURE = -1~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StopMotion-response)))
  "Returns full string definition for message of type 'StopMotion-response"
  (cl:format cl:nil "industrial_msgs/ServiceReturnCode code~%~%~%================================================================================~%MSG: industrial_msgs/ServiceReturnCode~%# Service return codes for simple requests.  All ROS-Industrial service~%# replies are required to have a return code indicating success or failure~%# Specific return codes for different failure should be negative.~%int8 val~%~%int8 SUCCESS = 1~%int8 FAILURE = -1~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StopMotion-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'code))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StopMotion-response>))
  "Converts a ROS message object to a list"
  (cl:list 'StopMotion-response
    (cl:cons ':code (code msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'StopMotion)))
  'StopMotion-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'StopMotion)))
  'StopMotion-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StopMotion)))
  "Returns string type for a service object of type '<StopMotion>"
  "industrial_msgs/StopMotion")
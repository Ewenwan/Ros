; Auto-generated. Do not edit!


(cl:in-package industrial_msgs-srv)


;//! \htmlinclude StartMotion-request.msg.html

(cl:defclass <StartMotion-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass StartMotion-request (<StartMotion-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StartMotion-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StartMotion-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name industrial_msgs-srv:<StartMotion-request> is deprecated: use industrial_msgs-srv:StartMotion-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StartMotion-request>) ostream)
  "Serializes a message object of type '<StartMotion-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StartMotion-request>) istream)
  "Deserializes a message object of type '<StartMotion-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StartMotion-request>)))
  "Returns string type for a service object of type '<StartMotion-request>"
  "industrial_msgs/StartMotionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StartMotion-request)))
  "Returns string type for a service object of type 'StartMotion-request"
  "industrial_msgs/StartMotionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StartMotion-request>)))
  "Returns md5sum for a message object of type '<StartMotion-request>"
  "50b1f38f75f5677e5692f3b3e7e1ea48")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StartMotion-request)))
  "Returns md5sum for a message object of type 'StartMotion-request"
  "50b1f38f75f5677e5692f3b3e7e1ea48")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StartMotion-request>)))
  "Returns full string definition for message of type '<StartMotion-request>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StartMotion-request)))
  "Returns full string definition for message of type 'StartMotion-request"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StartMotion-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StartMotion-request>))
  "Converts a ROS message object to a list"
  (cl:list 'StartMotion-request
))
;//! \htmlinclude StartMotion-response.msg.html

(cl:defclass <StartMotion-response> (roslisp-msg-protocol:ros-message)
  ((code
    :reader code
    :initarg :code
    :type industrial_msgs-msg:ServiceReturnCode
    :initform (cl:make-instance 'industrial_msgs-msg:ServiceReturnCode)))
)

(cl:defclass StartMotion-response (<StartMotion-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StartMotion-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StartMotion-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name industrial_msgs-srv:<StartMotion-response> is deprecated: use industrial_msgs-srv:StartMotion-response instead.")))

(cl:ensure-generic-function 'code-val :lambda-list '(m))
(cl:defmethod code-val ((m <StartMotion-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader industrial_msgs-srv:code-val is deprecated.  Use industrial_msgs-srv:code instead.")
  (code m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StartMotion-response>) ostream)
  "Serializes a message object of type '<StartMotion-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'code) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StartMotion-response>) istream)
  "Deserializes a message object of type '<StartMotion-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'code) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StartMotion-response>)))
  "Returns string type for a service object of type '<StartMotion-response>"
  "industrial_msgs/StartMotionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StartMotion-response)))
  "Returns string type for a service object of type 'StartMotion-response"
  "industrial_msgs/StartMotionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StartMotion-response>)))
  "Returns md5sum for a message object of type '<StartMotion-response>"
  "50b1f38f75f5677e5692f3b3e7e1ea48")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StartMotion-response)))
  "Returns md5sum for a message object of type 'StartMotion-response"
  "50b1f38f75f5677e5692f3b3e7e1ea48")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StartMotion-response>)))
  "Returns full string definition for message of type '<StartMotion-response>"
  (cl:format cl:nil "industrial_msgs/ServiceReturnCode code~%~%~%================================================================================~%MSG: industrial_msgs/ServiceReturnCode~%# Service return codes for simple requests.  All ROS-Industrial service~%# replies are required to have a return code indicating success or failure~%# Specific return codes for different failure should be negative.~%int8 val~%~%int8 SUCCESS = 1~%int8 FAILURE = -1~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StartMotion-response)))
  "Returns full string definition for message of type 'StartMotion-response"
  (cl:format cl:nil "industrial_msgs/ServiceReturnCode code~%~%~%================================================================================~%MSG: industrial_msgs/ServiceReturnCode~%# Service return codes for simple requests.  All ROS-Industrial service~%# replies are required to have a return code indicating success or failure~%# Specific return codes for different failure should be negative.~%int8 val~%~%int8 SUCCESS = 1~%int8 FAILURE = -1~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StartMotion-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'code))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StartMotion-response>))
  "Converts a ROS message object to a list"
  (cl:list 'StartMotion-response
    (cl:cons ':code (code msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'StartMotion)))
  'StartMotion-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'StartMotion)))
  'StartMotion-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StartMotion)))
  "Returns string type for a service object of type '<StartMotion>"
  "industrial_msgs/StartMotion")
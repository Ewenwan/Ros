; Auto-generated. Do not edit!


(cl:in-package industrial_msgs-msg)


;//! \htmlinclude ServiceReturnCode.msg.html

(cl:defclass <ServiceReturnCode> (roslisp-msg-protocol:ros-message)
  ((val
    :reader val
    :initarg :val
    :type cl:fixnum
    :initform 0))
)

(cl:defclass ServiceReturnCode (<ServiceReturnCode>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ServiceReturnCode>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ServiceReturnCode)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name industrial_msgs-msg:<ServiceReturnCode> is deprecated: use industrial_msgs-msg:ServiceReturnCode instead.")))

(cl:ensure-generic-function 'val-val :lambda-list '(m))
(cl:defmethod val-val ((m <ServiceReturnCode>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader industrial_msgs-msg:val-val is deprecated.  Use industrial_msgs-msg:val instead.")
  (val m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<ServiceReturnCode>)))
    "Constants for message type '<ServiceReturnCode>"
  '((:SUCCESS . 1)
    (:FAILURE . -1))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'ServiceReturnCode)))
    "Constants for message type 'ServiceReturnCode"
  '((:SUCCESS . 1)
    (:FAILURE . -1))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ServiceReturnCode>) ostream)
  "Serializes a message object of type '<ServiceReturnCode>"
  (cl:let* ((signed (cl:slot-value msg 'val)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ServiceReturnCode>) istream)
  "Deserializes a message object of type '<ServiceReturnCode>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'val) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ServiceReturnCode>)))
  "Returns string type for a message object of type '<ServiceReturnCode>"
  "industrial_msgs/ServiceReturnCode")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ServiceReturnCode)))
  "Returns string type for a message object of type 'ServiceReturnCode"
  "industrial_msgs/ServiceReturnCode")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ServiceReturnCode>)))
  "Returns md5sum for a message object of type '<ServiceReturnCode>"
  "85a4e241266be66b1e1426b03083a412")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ServiceReturnCode)))
  "Returns md5sum for a message object of type 'ServiceReturnCode"
  "85a4e241266be66b1e1426b03083a412")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ServiceReturnCode>)))
  "Returns full string definition for message of type '<ServiceReturnCode>"
  (cl:format cl:nil "# Service return codes for simple requests.  All ROS-Industrial service~%# replies are required to have a return code indicating success or failure~%# Specific return codes for different failure should be negative.~%int8 val~%~%int8 SUCCESS = 1~%int8 FAILURE = -1~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ServiceReturnCode)))
  "Returns full string definition for message of type 'ServiceReturnCode"
  (cl:format cl:nil "# Service return codes for simple requests.  All ROS-Industrial service~%# replies are required to have a return code indicating success or failure~%# Specific return codes for different failure should be negative.~%int8 val~%~%int8 SUCCESS = 1~%int8 FAILURE = -1~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ServiceReturnCode>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ServiceReturnCode>))
  "Converts a ROS message object to a list"
  (cl:list 'ServiceReturnCode
    (cl:cons ':val (val msg))
))

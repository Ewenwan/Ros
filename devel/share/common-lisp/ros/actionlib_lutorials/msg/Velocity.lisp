; Auto-generated. Do not edit!


(cl:in-package actionlib_lutorials-msg)


;//! \htmlinclude Velocity.msg.html

(cl:defclass <Velocity> (roslisp-msg-protocol:ros-message)
  ((linear
    :reader linear
    :initarg :linear
    :type cl:float
    :initform 0.0)
   (angular
    :reader angular
    :initarg :angular
    :type cl:float
    :initform 0.0))
)

(cl:defclass Velocity (<Velocity>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Velocity>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Velocity)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name actionlib_lutorials-msg:<Velocity> is deprecated: use actionlib_lutorials-msg:Velocity instead.")))

(cl:ensure-generic-function 'linear-val :lambda-list '(m))
(cl:defmethod linear-val ((m <Velocity>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader actionlib_lutorials-msg:linear-val is deprecated.  Use actionlib_lutorials-msg:linear instead.")
  (linear m))

(cl:ensure-generic-function 'angular-val :lambda-list '(m))
(cl:defmethod angular-val ((m <Velocity>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader actionlib_lutorials-msg:angular-val is deprecated.  Use actionlib_lutorials-msg:angular instead.")
  (angular m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Velocity>) ostream)
  "Serializes a message object of type '<Velocity>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'linear))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'angular))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Velocity>) istream)
  "Deserializes a message object of type '<Velocity>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'linear) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angular) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Velocity>)))
  "Returns string type for a message object of type '<Velocity>"
  "actionlib_lutorials/Velocity")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Velocity)))
  "Returns string type for a message object of type 'Velocity"
  "actionlib_lutorials/Velocity")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Velocity>)))
  "Returns md5sum for a message object of type '<Velocity>"
  "9d5c2dcd348ac8f76ce2a4307bd63a13")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Velocity)))
  "Returns md5sum for a message object of type 'Velocity"
  "9d5c2dcd348ac8f76ce2a4307bd63a13")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Velocity>)))
  "Returns full string definition for message of type '<Velocity>"
  (cl:format cl:nil "float32 linear~%float32 angular~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Velocity)))
  "Returns full string definition for message of type 'Velocity"
  (cl:format cl:nil "float32 linear~%float32 angular~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Velocity>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Velocity>))
  "Converts a ROS message object to a list"
  (cl:list 'Velocity
    (cl:cons ':linear (linear msg))
    (cl:cons ':angular (angular msg))
))

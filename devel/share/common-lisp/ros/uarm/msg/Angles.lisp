; Auto-generated. Do not edit!


(cl:in-package uarm-msg)


;//! \htmlinclude Angles.msg.html

(cl:defclass <Angles> (roslisp-msg-protocol:ros-message)
  ((servo_1
    :reader servo_1
    :initarg :servo_1
    :type cl:fixnum
    :initform 0)
   (servo_2
    :reader servo_2
    :initarg :servo_2
    :type cl:fixnum
    :initform 0)
   (servo_3
    :reader servo_3
    :initarg :servo_3
    :type cl:fixnum
    :initform 0)
   (servo_4
    :reader servo_4
    :initarg :servo_4
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Angles (<Angles>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Angles>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Angles)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uarm-msg:<Angles> is deprecated: use uarm-msg:Angles instead.")))

(cl:ensure-generic-function 'servo_1-val :lambda-list '(m))
(cl:defmethod servo_1-val ((m <Angles>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uarm-msg:servo_1-val is deprecated.  Use uarm-msg:servo_1 instead.")
  (servo_1 m))

(cl:ensure-generic-function 'servo_2-val :lambda-list '(m))
(cl:defmethod servo_2-val ((m <Angles>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uarm-msg:servo_2-val is deprecated.  Use uarm-msg:servo_2 instead.")
  (servo_2 m))

(cl:ensure-generic-function 'servo_3-val :lambda-list '(m))
(cl:defmethod servo_3-val ((m <Angles>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uarm-msg:servo_3-val is deprecated.  Use uarm-msg:servo_3 instead.")
  (servo_3 m))

(cl:ensure-generic-function 'servo_4-val :lambda-list '(m))
(cl:defmethod servo_4-val ((m <Angles>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uarm-msg:servo_4-val is deprecated.  Use uarm-msg:servo_4 instead.")
  (servo_4 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Angles>) ostream)
  "Serializes a message object of type '<Angles>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'servo_1)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'servo_2)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'servo_3)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'servo_4)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Angles>) istream)
  "Deserializes a message object of type '<Angles>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'servo_1)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'servo_2)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'servo_3)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'servo_4)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Angles>)))
  "Returns string type for a message object of type '<Angles>"
  "uarm/Angles")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Angles)))
  "Returns string type for a message object of type 'Angles"
  "uarm/Angles")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Angles>)))
  "Returns md5sum for a message object of type '<Angles>"
  "c365f7c0f52d109b4b516ca588b94bd3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Angles)))
  "Returns md5sum for a message object of type 'Angles"
  "c365f7c0f52d109b4b516ca588b94bd3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Angles>)))
  "Returns full string definition for message of type '<Angles>"
  (cl:format cl:nil "uint8 servo_1~%uint8 servo_2~%uint8 servo_3~%uint8 servo_4~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Angles)))
  "Returns full string definition for message of type 'Angles"
  (cl:format cl:nil "uint8 servo_1~%uint8 servo_2~%uint8 servo_3~%uint8 servo_4~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Angles>))
  (cl:+ 0
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Angles>))
  "Converts a ROS message object to a list"
  (cl:list 'Angles
    (cl:cons ':servo_1 (servo_1 msg))
    (cl:cons ':servo_2 (servo_2 msg))
    (cl:cons ':servo_3 (servo_3 msg))
    (cl:cons ':servo_4 (servo_4 msg))
))

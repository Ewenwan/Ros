; Auto-generated. Do not edit!


(cl:in-package uarm-msg)


;//! \htmlinclude CoordsWithTS4.msg.html

(cl:defclass <CoordsWithTS4> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0)
   (z
    :reader z
    :initarg :z
    :type cl:float
    :initform 0.0)
   (time
    :reader time
    :initarg :time
    :type cl:fixnum
    :initform 0)
   (servo_4
    :reader servo_4
    :initarg :servo_4
    :type cl:fixnum
    :initform 0))
)

(cl:defclass CoordsWithTS4 (<CoordsWithTS4>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CoordsWithTS4>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CoordsWithTS4)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uarm-msg:<CoordsWithTS4> is deprecated: use uarm-msg:CoordsWithTS4 instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <CoordsWithTS4>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uarm-msg:x-val is deprecated.  Use uarm-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <CoordsWithTS4>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uarm-msg:y-val is deprecated.  Use uarm-msg:y instead.")
  (y m))

(cl:ensure-generic-function 'z-val :lambda-list '(m))
(cl:defmethod z-val ((m <CoordsWithTS4>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uarm-msg:z-val is deprecated.  Use uarm-msg:z instead.")
  (z m))

(cl:ensure-generic-function 'time-val :lambda-list '(m))
(cl:defmethod time-val ((m <CoordsWithTS4>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uarm-msg:time-val is deprecated.  Use uarm-msg:time instead.")
  (time m))

(cl:ensure-generic-function 'servo_4-val :lambda-list '(m))
(cl:defmethod servo_4-val ((m <CoordsWithTS4>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uarm-msg:servo_4-val is deprecated.  Use uarm-msg:servo_4 instead.")
  (servo_4 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CoordsWithTS4>) ostream)
  "Serializes a message object of type '<CoordsWithTS4>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'time)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'servo_4)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CoordsWithTS4>) istream)
  "Deserializes a message object of type '<CoordsWithTS4>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'z) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'time)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'servo_4)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CoordsWithTS4>)))
  "Returns string type for a message object of type '<CoordsWithTS4>"
  "uarm/CoordsWithTS4")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CoordsWithTS4)))
  "Returns string type for a message object of type 'CoordsWithTS4"
  "uarm/CoordsWithTS4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CoordsWithTS4>)))
  "Returns md5sum for a message object of type '<CoordsWithTS4>"
  "d591a4c1754daa954bee92aa3a0cadd7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CoordsWithTS4)))
  "Returns md5sum for a message object of type 'CoordsWithTS4"
  "d591a4c1754daa954bee92aa3a0cadd7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CoordsWithTS4>)))
  "Returns full string definition for message of type '<CoordsWithTS4>"
  (cl:format cl:nil "float32 x~%float32 y~%float32 z~%uint8 time~%uint8 servo_4~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CoordsWithTS4)))
  "Returns full string definition for message of type 'CoordsWithTS4"
  (cl:format cl:nil "float32 x~%float32 y~%float32 z~%uint8 time~%uint8 servo_4~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CoordsWithTS4>))
  (cl:+ 0
     4
     4
     4
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CoordsWithTS4>))
  "Converts a ROS message object to a list"
  (cl:list 'CoordsWithTS4
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':z (z msg))
    (cl:cons ':time (time msg))
    (cl:cons ':servo_4 (servo_4 msg))
))

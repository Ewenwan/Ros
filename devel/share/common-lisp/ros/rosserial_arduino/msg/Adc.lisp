; Auto-generated. Do not edit!


(cl:in-package rosserial_arduino-msg)


;//! \htmlinclude Adc.msg.html

(cl:defclass <Adc> (roslisp-msg-protocol:ros-message)
  ((adc0
    :reader adc0
    :initarg :adc0
    :type cl:fixnum
    :initform 0)
   (adc1
    :reader adc1
    :initarg :adc1
    :type cl:fixnum
    :initform 0)
   (adc2
    :reader adc2
    :initarg :adc2
    :type cl:fixnum
    :initform 0)
   (adc3
    :reader adc3
    :initarg :adc3
    :type cl:fixnum
    :initform 0)
   (adc4
    :reader adc4
    :initarg :adc4
    :type cl:fixnum
    :initform 0)
   (adc5
    :reader adc5
    :initarg :adc5
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Adc (<Adc>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Adc>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Adc)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rosserial_arduino-msg:<Adc> is deprecated: use rosserial_arduino-msg:Adc instead.")))

(cl:ensure-generic-function 'adc0-val :lambda-list '(m))
(cl:defmethod adc0-val ((m <Adc>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rosserial_arduino-msg:adc0-val is deprecated.  Use rosserial_arduino-msg:adc0 instead.")
  (adc0 m))

(cl:ensure-generic-function 'adc1-val :lambda-list '(m))
(cl:defmethod adc1-val ((m <Adc>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rosserial_arduino-msg:adc1-val is deprecated.  Use rosserial_arduino-msg:adc1 instead.")
  (adc1 m))

(cl:ensure-generic-function 'adc2-val :lambda-list '(m))
(cl:defmethod adc2-val ((m <Adc>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rosserial_arduino-msg:adc2-val is deprecated.  Use rosserial_arduino-msg:adc2 instead.")
  (adc2 m))

(cl:ensure-generic-function 'adc3-val :lambda-list '(m))
(cl:defmethod adc3-val ((m <Adc>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rosserial_arduino-msg:adc3-val is deprecated.  Use rosserial_arduino-msg:adc3 instead.")
  (adc3 m))

(cl:ensure-generic-function 'adc4-val :lambda-list '(m))
(cl:defmethod adc4-val ((m <Adc>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rosserial_arduino-msg:adc4-val is deprecated.  Use rosserial_arduino-msg:adc4 instead.")
  (adc4 m))

(cl:ensure-generic-function 'adc5-val :lambda-list '(m))
(cl:defmethod adc5-val ((m <Adc>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rosserial_arduino-msg:adc5-val is deprecated.  Use rosserial_arduino-msg:adc5 instead.")
  (adc5 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Adc>) ostream)
  "Serializes a message object of type '<Adc>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'adc0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'adc0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'adc1)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'adc1)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'adc2)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'adc2)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'adc3)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'adc3)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'adc4)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'adc4)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'adc5)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'adc5)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Adc>) istream)
  "Deserializes a message object of type '<Adc>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'adc0)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'adc0)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'adc1)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'adc1)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'adc2)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'adc2)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'adc3)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'adc3)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'adc4)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'adc4)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'adc5)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'adc5)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Adc>)))
  "Returns string type for a message object of type '<Adc>"
  "rosserial_arduino/Adc")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Adc)))
  "Returns string type for a message object of type 'Adc"
  "rosserial_arduino/Adc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Adc>)))
  "Returns md5sum for a message object of type '<Adc>"
  "6d7853a614e2e821319068311f2af25b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Adc)))
  "Returns md5sum for a message object of type 'Adc"
  "6d7853a614e2e821319068311f2af25b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Adc>)))
  "Returns full string definition for message of type '<Adc>"
  (cl:format cl:nil "uint16 adc0~%uint16 adc1~%uint16 adc2~%uint16 adc3~%uint16 adc4~%uint16 adc5~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Adc)))
  "Returns full string definition for message of type 'Adc"
  (cl:format cl:nil "uint16 adc0~%uint16 adc1~%uint16 adc2~%uint16 adc3~%uint16 adc4~%uint16 adc5~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Adc>))
  (cl:+ 0
     2
     2
     2
     2
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Adc>))
  "Converts a ROS message object to a list"
  (cl:list 'Adc
    (cl:cons ':adc0 (adc0 msg))
    (cl:cons ':adc1 (adc1 msg))
    (cl:cons ':adc2 (adc2 msg))
    (cl:cons ':adc3 (adc3 msg))
    (cl:cons ':adc4 (adc4 msg))
    (cl:cons ':adc5 (adc5 msg))
))

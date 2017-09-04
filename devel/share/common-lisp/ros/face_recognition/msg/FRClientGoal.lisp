; Auto-generated. Do not edit!


(cl:in-package face_recognition-msg)


;//! \htmlinclude FRClientGoal.msg.html

(cl:defclass <FRClientGoal> (roslisp-msg-protocol:ros-message)
  ((order_id
    :reader order_id
    :initarg :order_id
    :type cl:fixnum
    :initform 0)
   (order_argument
    :reader order_argument
    :initarg :order_argument
    :type cl:string
    :initform ""))
)

(cl:defclass FRClientGoal (<FRClientGoal>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FRClientGoal>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FRClientGoal)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name face_recognition-msg:<FRClientGoal> is deprecated: use face_recognition-msg:FRClientGoal instead.")))

(cl:ensure-generic-function 'order_id-val :lambda-list '(m))
(cl:defmethod order_id-val ((m <FRClientGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader face_recognition-msg:order_id-val is deprecated.  Use face_recognition-msg:order_id instead.")
  (order_id m))

(cl:ensure-generic-function 'order_argument-val :lambda-list '(m))
(cl:defmethod order_argument-val ((m <FRClientGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader face_recognition-msg:order_argument-val is deprecated.  Use face_recognition-msg:order_argument instead.")
  (order_argument m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FRClientGoal>) ostream)
  "Serializes a message object of type '<FRClientGoal>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'order_id)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'order_argument))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'order_argument))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FRClientGoal>) istream)
  "Deserializes a message object of type '<FRClientGoal>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'order_id)) (cl:read-byte istream))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'order_argument) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'order_argument) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FRClientGoal>)))
  "Returns string type for a message object of type '<FRClientGoal>"
  "face_recognition/FRClientGoal")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FRClientGoal)))
  "Returns string type for a message object of type 'FRClientGoal"
  "face_recognition/FRClientGoal")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FRClientGoal>)))
  "Returns md5sum for a message object of type '<FRClientGoal>"
  "12fa0d0af9b141eceafa42011d31f9c2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FRClientGoal)))
  "Returns md5sum for a message object of type 'FRClientGoal"
  "12fa0d0af9b141eceafa42011d31f9c2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FRClientGoal>)))
  "Returns full string definition for message of type '<FRClientGoal>"
  (cl:format cl:nil "uint8   order_id ~%string  order_argument~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FRClientGoal)))
  "Returns full string definition for message of type 'FRClientGoal"
  (cl:format cl:nil "uint8   order_id ~%string  order_argument~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FRClientGoal>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'order_argument))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FRClientGoal>))
  "Converts a ROS message object to a list"
  (cl:list 'FRClientGoal
    (cl:cons ':order_id (order_id msg))
    (cl:cons ':order_argument (order_argument msg))
))

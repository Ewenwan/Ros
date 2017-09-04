; Auto-generated. Do not edit!


(cl:in-package rosserial_msgs-msg)


;//! \htmlinclude Log.msg.html

(cl:defclass <Log> (roslisp-msg-protocol:ros-message)
  ((level
    :reader level
    :initarg :level
    :type cl:fixnum
    :initform 0)
   (msg
    :reader msg
    :initarg :msg
    :type cl:string
    :initform ""))
)

(cl:defclass Log (<Log>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Log>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Log)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rosserial_msgs-msg:<Log> is deprecated: use rosserial_msgs-msg:Log instead.")))

(cl:ensure-generic-function 'level-val :lambda-list '(m))
(cl:defmethod level-val ((m <Log>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rosserial_msgs-msg:level-val is deprecated.  Use rosserial_msgs-msg:level instead.")
  (level m))

(cl:ensure-generic-function 'msg-val :lambda-list '(m))
(cl:defmethod msg-val ((m <Log>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rosserial_msgs-msg:msg-val is deprecated.  Use rosserial_msgs-msg:msg instead.")
  (msg m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<Log>)))
    "Constants for message type '<Log>"
  '((:ROSDEBUG . 0)
    (:INFO . 1)
    (:WARN . 2)
    (:ERROR . 3)
    (:FATAL . 4))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'Log)))
    "Constants for message type 'Log"
  '((:ROSDEBUG . 0)
    (:INFO . 1)
    (:WARN . 2)
    (:ERROR . 3)
    (:FATAL . 4))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Log>) ostream)
  "Serializes a message object of type '<Log>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'level)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'msg))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'msg))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Log>) istream)
  "Deserializes a message object of type '<Log>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'level)) (cl:read-byte istream))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'msg) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'msg) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Log>)))
  "Returns string type for a message object of type '<Log>"
  "rosserial_msgs/Log")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Log)))
  "Returns string type for a message object of type 'Log"
  "rosserial_msgs/Log")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Log>)))
  "Returns md5sum for a message object of type '<Log>"
  "11abd731c25933261cd6183bd12d6295")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Log)))
  "Returns md5sum for a message object of type 'Log"
  "11abd731c25933261cd6183bd12d6295")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Log>)))
  "Returns full string definition for message of type '<Log>"
  (cl:format cl:nil "~%#ROS Logging Levels~%uint8 ROSDEBUG=0~%uint8 INFO=1~%uint8 WARN=2~%uint8 ERROR=3~%uint8 FATAL=4~%~%uint8 level~%string msg~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Log)))
  "Returns full string definition for message of type 'Log"
  (cl:format cl:nil "~%#ROS Logging Levels~%uint8 ROSDEBUG=0~%uint8 INFO=1~%uint8 WARN=2~%uint8 ERROR=3~%uint8 FATAL=4~%~%uint8 level~%string msg~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Log>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'msg))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Log>))
  "Converts a ROS message object to a list"
  (cl:list 'Log
    (cl:cons ':level (level msg))
    (cl:cons ':msg (msg msg))
))

; Auto-generated. Do not edit!


(cl:in-package rosserial_msgs-msg)


;//! \htmlinclude TopicInfo.msg.html

(cl:defclass <TopicInfo> (roslisp-msg-protocol:ros-message)
  ((topic_id
    :reader topic_id
    :initarg :topic_id
    :type cl:fixnum
    :initform 0)
   (topic_name
    :reader topic_name
    :initarg :topic_name
    :type cl:string
    :initform "")
   (message_type
    :reader message_type
    :initarg :message_type
    :type cl:string
    :initform "")
   (md5sum
    :reader md5sum
    :initarg :md5sum
    :type cl:string
    :initform "")
   (buffer_size
    :reader buffer_size
    :initarg :buffer_size
    :type cl:integer
    :initform 0))
)

(cl:defclass TopicInfo (<TopicInfo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TopicInfo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TopicInfo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rosserial_msgs-msg:<TopicInfo> is deprecated: use rosserial_msgs-msg:TopicInfo instead.")))

(cl:ensure-generic-function 'topic_id-val :lambda-list '(m))
(cl:defmethod topic_id-val ((m <TopicInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rosserial_msgs-msg:topic_id-val is deprecated.  Use rosserial_msgs-msg:topic_id instead.")
  (topic_id m))

(cl:ensure-generic-function 'topic_name-val :lambda-list '(m))
(cl:defmethod topic_name-val ((m <TopicInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rosserial_msgs-msg:topic_name-val is deprecated.  Use rosserial_msgs-msg:topic_name instead.")
  (topic_name m))

(cl:ensure-generic-function 'message_type-val :lambda-list '(m))
(cl:defmethod message_type-val ((m <TopicInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rosserial_msgs-msg:message_type-val is deprecated.  Use rosserial_msgs-msg:message_type instead.")
  (message_type m))

(cl:ensure-generic-function 'md5sum-val :lambda-list '(m))
(cl:defmethod md5sum-val ((m <TopicInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rosserial_msgs-msg:md5sum-val is deprecated.  Use rosserial_msgs-msg:md5sum instead.")
  (md5sum m))

(cl:ensure-generic-function 'buffer_size-val :lambda-list '(m))
(cl:defmethod buffer_size-val ((m <TopicInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rosserial_msgs-msg:buffer_size-val is deprecated.  Use rosserial_msgs-msg:buffer_size instead.")
  (buffer_size m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<TopicInfo>)))
    "Constants for message type '<TopicInfo>"
  '((:ID_PUBLISHER . 0)
    (:ID_SUBSCRIBER . 1)
    (:ID_SERVICE_SERVER . 2)
    (:ID_SERVICE_CLIENT . 4)
    (:ID_PARAMETER_REQUEST . 6)
    (:ID_LOG . 7)
    (:ID_TIME . 10)
    (:ID_TX_STOP . 11))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'TopicInfo)))
    "Constants for message type 'TopicInfo"
  '((:ID_PUBLISHER . 0)
    (:ID_SUBSCRIBER . 1)
    (:ID_SERVICE_SERVER . 2)
    (:ID_SERVICE_CLIENT . 4)
    (:ID_PARAMETER_REQUEST . 6)
    (:ID_LOG . 7)
    (:ID_TIME . 10)
    (:ID_TX_STOP . 11))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TopicInfo>) ostream)
  "Serializes a message object of type '<TopicInfo>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'topic_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'topic_id)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'topic_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'topic_name))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message_type))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message_type))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'md5sum))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'md5sum))
  (cl:let* ((signed (cl:slot-value msg 'buffer_size)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TopicInfo>) istream)
  "Deserializes a message object of type '<TopicInfo>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'topic_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'topic_id)) (cl:read-byte istream))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'topic_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'topic_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'message_type) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'message_type) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'md5sum) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'md5sum) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'buffer_size) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TopicInfo>)))
  "Returns string type for a message object of type '<TopicInfo>"
  "rosserial_msgs/TopicInfo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TopicInfo)))
  "Returns string type for a message object of type 'TopicInfo"
  "rosserial_msgs/TopicInfo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TopicInfo>)))
  "Returns md5sum for a message object of type '<TopicInfo>"
  "0ad51f88fc44892f8c10684077646005")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TopicInfo)))
  "Returns md5sum for a message object of type 'TopicInfo"
  "0ad51f88fc44892f8c10684077646005")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TopicInfo>)))
  "Returns full string definition for message of type '<TopicInfo>"
  (cl:format cl:nil "# special topic_ids~%uint16 ID_PUBLISHER=0~%uint16 ID_SUBSCRIBER=1~%uint16 ID_SERVICE_SERVER=2~%uint16 ID_SERVICE_CLIENT=4~%uint16 ID_PARAMETER_REQUEST=6~%uint16 ID_LOG=7~%uint16 ID_TIME=10~%uint16 ID_TX_STOP=11~%~%# The endpoint ID for this topic~%uint16 topic_id~%~%string topic_name~%string message_type~%~%# MD5 checksum for this message type~%string md5sum~%~%# size of the buffer message must fit in~%int32 buffer_size~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TopicInfo)))
  "Returns full string definition for message of type 'TopicInfo"
  (cl:format cl:nil "# special topic_ids~%uint16 ID_PUBLISHER=0~%uint16 ID_SUBSCRIBER=1~%uint16 ID_SERVICE_SERVER=2~%uint16 ID_SERVICE_CLIENT=4~%uint16 ID_PARAMETER_REQUEST=6~%uint16 ID_LOG=7~%uint16 ID_TIME=10~%uint16 ID_TX_STOP=11~%~%# The endpoint ID for this topic~%uint16 topic_id~%~%string topic_name~%string message_type~%~%# MD5 checksum for this message type~%string md5sum~%~%# size of the buffer message must fit in~%int32 buffer_size~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TopicInfo>))
  (cl:+ 0
     2
     4 (cl:length (cl:slot-value msg 'topic_name))
     4 (cl:length (cl:slot-value msg 'message_type))
     4 (cl:length (cl:slot-value msg 'md5sum))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TopicInfo>))
  "Converts a ROS message object to a list"
  (cl:list 'TopicInfo
    (cl:cons ':topic_id (topic_id msg))
    (cl:cons ':topic_name (topic_name msg))
    (cl:cons ':message_type (message_type msg))
    (cl:cons ':md5sum (md5sum msg))
    (cl:cons ':buffer_size (buffer_size msg))
))

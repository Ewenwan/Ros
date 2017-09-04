; Auto-generated. Do not edit!


(cl:in-package actionlib_lutorials-msg)


;//! \htmlinclude AveragingFeedback.msg.html

(cl:defclass <AveragingFeedback> (roslisp-msg-protocol:ros-message)
  ((sample
    :reader sample
    :initarg :sample
    :type cl:integer
    :initform 0)
   (data
    :reader data
    :initarg :data
    :type cl:float
    :initform 0.0)
   (mean
    :reader mean
    :initarg :mean
    :type cl:float
    :initform 0.0)
   (std_dev
    :reader std_dev
    :initarg :std_dev
    :type cl:float
    :initform 0.0))
)

(cl:defclass AveragingFeedback (<AveragingFeedback>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AveragingFeedback>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AveragingFeedback)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name actionlib_lutorials-msg:<AveragingFeedback> is deprecated: use actionlib_lutorials-msg:AveragingFeedback instead.")))

(cl:ensure-generic-function 'sample-val :lambda-list '(m))
(cl:defmethod sample-val ((m <AveragingFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader actionlib_lutorials-msg:sample-val is deprecated.  Use actionlib_lutorials-msg:sample instead.")
  (sample m))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <AveragingFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader actionlib_lutorials-msg:data-val is deprecated.  Use actionlib_lutorials-msg:data instead.")
  (data m))

(cl:ensure-generic-function 'mean-val :lambda-list '(m))
(cl:defmethod mean-val ((m <AveragingFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader actionlib_lutorials-msg:mean-val is deprecated.  Use actionlib_lutorials-msg:mean instead.")
  (mean m))

(cl:ensure-generic-function 'std_dev-val :lambda-list '(m))
(cl:defmethod std_dev-val ((m <AveragingFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader actionlib_lutorials-msg:std_dev-val is deprecated.  Use actionlib_lutorials-msg:std_dev instead.")
  (std_dev m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AveragingFeedback>) ostream)
  "Serializes a message object of type '<AveragingFeedback>"
  (cl:let* ((signed (cl:slot-value msg 'sample)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'mean))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'std_dev))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AveragingFeedback>) istream)
  "Deserializes a message object of type '<AveragingFeedback>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'sample) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'data) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'mean) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'std_dev) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AveragingFeedback>)))
  "Returns string type for a message object of type '<AveragingFeedback>"
  "actionlib_lutorials/AveragingFeedback")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AveragingFeedback)))
  "Returns string type for a message object of type 'AveragingFeedback"
  "actionlib_lutorials/AveragingFeedback")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AveragingFeedback>)))
  "Returns md5sum for a message object of type '<AveragingFeedback>"
  "9e8dfc53c2f2a032ca33fa80ec46fd4f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AveragingFeedback)))
  "Returns md5sum for a message object of type 'AveragingFeedback"
  "9e8dfc53c2f2a032ca33fa80ec46fd4f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AveragingFeedback>)))
  "Returns full string definition for message of type '<AveragingFeedback>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#feedback          定义反馈 话题类型 ~%int32 sample~%float32 data~%float32 mean~%float32 std_dev~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AveragingFeedback)))
  "Returns full string definition for message of type 'AveragingFeedback"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#feedback          定义反馈 话题类型 ~%int32 sample~%float32 data~%float32 mean~%float32 std_dev~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AveragingFeedback>))
  (cl:+ 0
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AveragingFeedback>))
  "Converts a ROS message object to a list"
  (cl:list 'AveragingFeedback
    (cl:cons ':sample (sample msg))
    (cl:cons ':data (data msg))
    (cl:cons ':mean (mean msg))
    (cl:cons ':std_dev (std_dev msg))
))

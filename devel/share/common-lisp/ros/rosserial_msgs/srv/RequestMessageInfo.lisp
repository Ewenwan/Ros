; Auto-generated. Do not edit!


(cl:in-package rosserial_msgs-srv)


;//! \htmlinclude RequestMessageInfo-request.msg.html

(cl:defclass <RequestMessageInfo-request> (roslisp-msg-protocol:ros-message)
  ((type
    :reader type
    :initarg :type
    :type cl:string
    :initform ""))
)

(cl:defclass RequestMessageInfo-request (<RequestMessageInfo-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RequestMessageInfo-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RequestMessageInfo-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rosserial_msgs-srv:<RequestMessageInfo-request> is deprecated: use rosserial_msgs-srv:RequestMessageInfo-request instead.")))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <RequestMessageInfo-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rosserial_msgs-srv:type-val is deprecated.  Use rosserial_msgs-srv:type instead.")
  (type m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RequestMessageInfo-request>) ostream)
  "Serializes a message object of type '<RequestMessageInfo-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'type))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'type))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RequestMessageInfo-request>) istream)
  "Deserializes a message object of type '<RequestMessageInfo-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'type) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'type) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RequestMessageInfo-request>)))
  "Returns string type for a service object of type '<RequestMessageInfo-request>"
  "rosserial_msgs/RequestMessageInfoRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RequestMessageInfo-request)))
  "Returns string type for a service object of type 'RequestMessageInfo-request"
  "rosserial_msgs/RequestMessageInfoRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RequestMessageInfo-request>)))
  "Returns md5sum for a message object of type '<RequestMessageInfo-request>"
  "6416d80296dfbbdd5f7b2cee839f9316")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RequestMessageInfo-request)))
  "Returns md5sum for a message object of type 'RequestMessageInfo-request"
  "6416d80296dfbbdd5f7b2cee839f9316")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RequestMessageInfo-request>)))
  "Returns full string definition for message of type '<RequestMessageInfo-request>"
  (cl:format cl:nil "~%string type~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RequestMessageInfo-request)))
  "Returns full string definition for message of type 'RequestMessageInfo-request"
  (cl:format cl:nil "~%string type~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RequestMessageInfo-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'type))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RequestMessageInfo-request>))
  "Converts a ROS message object to a list"
  (cl:list 'RequestMessageInfo-request
    (cl:cons ':type (type msg))
))
;//! \htmlinclude RequestMessageInfo-response.msg.html

(cl:defclass <RequestMessageInfo-response> (roslisp-msg-protocol:ros-message)
  ((md5
    :reader md5
    :initarg :md5
    :type cl:string
    :initform "")
   (definition
    :reader definition
    :initarg :definition
    :type cl:string
    :initform ""))
)

(cl:defclass RequestMessageInfo-response (<RequestMessageInfo-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RequestMessageInfo-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RequestMessageInfo-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rosserial_msgs-srv:<RequestMessageInfo-response> is deprecated: use rosserial_msgs-srv:RequestMessageInfo-response instead.")))

(cl:ensure-generic-function 'md5-val :lambda-list '(m))
(cl:defmethod md5-val ((m <RequestMessageInfo-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rosserial_msgs-srv:md5-val is deprecated.  Use rosserial_msgs-srv:md5 instead.")
  (md5 m))

(cl:ensure-generic-function 'definition-val :lambda-list '(m))
(cl:defmethod definition-val ((m <RequestMessageInfo-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rosserial_msgs-srv:definition-val is deprecated.  Use rosserial_msgs-srv:definition instead.")
  (definition m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RequestMessageInfo-response>) ostream)
  "Serializes a message object of type '<RequestMessageInfo-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'md5))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'md5))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'definition))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'definition))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RequestMessageInfo-response>) istream)
  "Deserializes a message object of type '<RequestMessageInfo-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'md5) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'md5) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'definition) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'definition) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RequestMessageInfo-response>)))
  "Returns string type for a service object of type '<RequestMessageInfo-response>"
  "rosserial_msgs/RequestMessageInfoResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RequestMessageInfo-response)))
  "Returns string type for a service object of type 'RequestMessageInfo-response"
  "rosserial_msgs/RequestMessageInfoResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RequestMessageInfo-response>)))
  "Returns md5sum for a message object of type '<RequestMessageInfo-response>"
  "6416d80296dfbbdd5f7b2cee839f9316")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RequestMessageInfo-response)))
  "Returns md5sum for a message object of type 'RequestMessageInfo-response"
  "6416d80296dfbbdd5f7b2cee839f9316")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RequestMessageInfo-response>)))
  "Returns full string definition for message of type '<RequestMessageInfo-response>"
  (cl:format cl:nil "~%~%string md5~%string definition~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RequestMessageInfo-response)))
  "Returns full string definition for message of type 'RequestMessageInfo-response"
  (cl:format cl:nil "~%~%string md5~%string definition~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RequestMessageInfo-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'md5))
     4 (cl:length (cl:slot-value msg 'definition))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RequestMessageInfo-response>))
  "Converts a ROS message object to a list"
  (cl:list 'RequestMessageInfo-response
    (cl:cons ':md5 (md5 msg))
    (cl:cons ':definition (definition msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'RequestMessageInfo)))
  'RequestMessageInfo-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'RequestMessageInfo)))
  'RequestMessageInfo-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RequestMessageInfo)))
  "Returns string type for a service object of type '<RequestMessageInfo>"
  "rosserial_msgs/RequestMessageInfo")
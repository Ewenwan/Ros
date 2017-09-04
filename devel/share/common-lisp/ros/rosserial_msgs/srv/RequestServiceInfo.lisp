; Auto-generated. Do not edit!


(cl:in-package rosserial_msgs-srv)


;//! \htmlinclude RequestServiceInfo-request.msg.html

(cl:defclass <RequestServiceInfo-request> (roslisp-msg-protocol:ros-message)
  ((service
    :reader service
    :initarg :service
    :type cl:string
    :initform ""))
)

(cl:defclass RequestServiceInfo-request (<RequestServiceInfo-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RequestServiceInfo-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RequestServiceInfo-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rosserial_msgs-srv:<RequestServiceInfo-request> is deprecated: use rosserial_msgs-srv:RequestServiceInfo-request instead.")))

(cl:ensure-generic-function 'service-val :lambda-list '(m))
(cl:defmethod service-val ((m <RequestServiceInfo-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rosserial_msgs-srv:service-val is deprecated.  Use rosserial_msgs-srv:service instead.")
  (service m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RequestServiceInfo-request>) ostream)
  "Serializes a message object of type '<RequestServiceInfo-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'service))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'service))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RequestServiceInfo-request>) istream)
  "Deserializes a message object of type '<RequestServiceInfo-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'service) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'service) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RequestServiceInfo-request>)))
  "Returns string type for a service object of type '<RequestServiceInfo-request>"
  "rosserial_msgs/RequestServiceInfoRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RequestServiceInfo-request)))
  "Returns string type for a service object of type 'RequestServiceInfo-request"
  "rosserial_msgs/RequestServiceInfoRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RequestServiceInfo-request>)))
  "Returns md5sum for a message object of type '<RequestServiceInfo-request>"
  "0961604b984b94b0b68e8074882be071")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RequestServiceInfo-request)))
  "Returns md5sum for a message object of type 'RequestServiceInfo-request"
  "0961604b984b94b0b68e8074882be071")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RequestServiceInfo-request>)))
  "Returns full string definition for message of type '<RequestServiceInfo-request>"
  (cl:format cl:nil "~%string service~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RequestServiceInfo-request)))
  "Returns full string definition for message of type 'RequestServiceInfo-request"
  (cl:format cl:nil "~%string service~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RequestServiceInfo-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'service))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RequestServiceInfo-request>))
  "Converts a ROS message object to a list"
  (cl:list 'RequestServiceInfo-request
    (cl:cons ':service (service msg))
))
;//! \htmlinclude RequestServiceInfo-response.msg.html

(cl:defclass <RequestServiceInfo-response> (roslisp-msg-protocol:ros-message)
  ((service_md5
    :reader service_md5
    :initarg :service_md5
    :type cl:string
    :initform "")
   (request_md5
    :reader request_md5
    :initarg :request_md5
    :type cl:string
    :initform "")
   (response_md5
    :reader response_md5
    :initarg :response_md5
    :type cl:string
    :initform ""))
)

(cl:defclass RequestServiceInfo-response (<RequestServiceInfo-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RequestServiceInfo-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RequestServiceInfo-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rosserial_msgs-srv:<RequestServiceInfo-response> is deprecated: use rosserial_msgs-srv:RequestServiceInfo-response instead.")))

(cl:ensure-generic-function 'service_md5-val :lambda-list '(m))
(cl:defmethod service_md5-val ((m <RequestServiceInfo-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rosserial_msgs-srv:service_md5-val is deprecated.  Use rosserial_msgs-srv:service_md5 instead.")
  (service_md5 m))

(cl:ensure-generic-function 'request_md5-val :lambda-list '(m))
(cl:defmethod request_md5-val ((m <RequestServiceInfo-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rosserial_msgs-srv:request_md5-val is deprecated.  Use rosserial_msgs-srv:request_md5 instead.")
  (request_md5 m))

(cl:ensure-generic-function 'response_md5-val :lambda-list '(m))
(cl:defmethod response_md5-val ((m <RequestServiceInfo-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rosserial_msgs-srv:response_md5-val is deprecated.  Use rosserial_msgs-srv:response_md5 instead.")
  (response_md5 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RequestServiceInfo-response>) ostream)
  "Serializes a message object of type '<RequestServiceInfo-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'service_md5))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'service_md5))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'request_md5))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'request_md5))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'response_md5))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'response_md5))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RequestServiceInfo-response>) istream)
  "Deserializes a message object of type '<RequestServiceInfo-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'service_md5) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'service_md5) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'request_md5) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'request_md5) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'response_md5) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'response_md5) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RequestServiceInfo-response>)))
  "Returns string type for a service object of type '<RequestServiceInfo-response>"
  "rosserial_msgs/RequestServiceInfoResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RequestServiceInfo-response)))
  "Returns string type for a service object of type 'RequestServiceInfo-response"
  "rosserial_msgs/RequestServiceInfoResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RequestServiceInfo-response>)))
  "Returns md5sum for a message object of type '<RequestServiceInfo-response>"
  "0961604b984b94b0b68e8074882be071")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RequestServiceInfo-response)))
  "Returns md5sum for a message object of type 'RequestServiceInfo-response"
  "0961604b984b94b0b68e8074882be071")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RequestServiceInfo-response>)))
  "Returns full string definition for message of type '<RequestServiceInfo-response>"
  (cl:format cl:nil "string service_md5~%string request_md5~%string response_md5~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RequestServiceInfo-response)))
  "Returns full string definition for message of type 'RequestServiceInfo-response"
  (cl:format cl:nil "string service_md5~%string request_md5~%string response_md5~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RequestServiceInfo-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'service_md5))
     4 (cl:length (cl:slot-value msg 'request_md5))
     4 (cl:length (cl:slot-value msg 'response_md5))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RequestServiceInfo-response>))
  "Converts a ROS message object to a list"
  (cl:list 'RequestServiceInfo-response
    (cl:cons ':service_md5 (service_md5 msg))
    (cl:cons ':request_md5 (request_md5 msg))
    (cl:cons ':response_md5 (response_md5 msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'RequestServiceInfo)))
  'RequestServiceInfo-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'RequestServiceInfo)))
  'RequestServiceInfo-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RequestServiceInfo)))
  "Returns string type for a service object of type '<RequestServiceInfo>"
  "rosserial_msgs/RequestServiceInfo")
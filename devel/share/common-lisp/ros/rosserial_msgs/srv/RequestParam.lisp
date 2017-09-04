; Auto-generated. Do not edit!


(cl:in-package rosserial_msgs-srv)


;//! \htmlinclude RequestParam-request.msg.html

(cl:defclass <RequestParam-request> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform ""))
)

(cl:defclass RequestParam-request (<RequestParam-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RequestParam-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RequestParam-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rosserial_msgs-srv:<RequestParam-request> is deprecated: use rosserial_msgs-srv:RequestParam-request instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <RequestParam-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rosserial_msgs-srv:name-val is deprecated.  Use rosserial_msgs-srv:name instead.")
  (name m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RequestParam-request>) ostream)
  "Serializes a message object of type '<RequestParam-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RequestParam-request>) istream)
  "Deserializes a message object of type '<RequestParam-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RequestParam-request>)))
  "Returns string type for a service object of type '<RequestParam-request>"
  "rosserial_msgs/RequestParamRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RequestParam-request)))
  "Returns string type for a service object of type 'RequestParam-request"
  "rosserial_msgs/RequestParamRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RequestParam-request>)))
  "Returns md5sum for a message object of type '<RequestParam-request>"
  "d7a0c2be00c9fd03cc69f2863de9c4d9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RequestParam-request)))
  "Returns md5sum for a message object of type 'RequestParam-request"
  "d7a0c2be00c9fd03cc69f2863de9c4d9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RequestParam-request>)))
  "Returns full string definition for message of type '<RequestParam-request>"
  (cl:format cl:nil "string name~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RequestParam-request)))
  "Returns full string definition for message of type 'RequestParam-request"
  (cl:format cl:nil "string name~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RequestParam-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RequestParam-request>))
  "Converts a ROS message object to a list"
  (cl:list 'RequestParam-request
    (cl:cons ':name (name msg))
))
;//! \htmlinclude RequestParam-response.msg.html

(cl:defclass <RequestParam-response> (roslisp-msg-protocol:ros-message)
  ((ints
    :reader ints
    :initarg :ints
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0))
   (floats
    :reader floats
    :initarg :floats
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (strings
    :reader strings
    :initarg :strings
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element "")))
)

(cl:defclass RequestParam-response (<RequestParam-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RequestParam-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RequestParam-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rosserial_msgs-srv:<RequestParam-response> is deprecated: use rosserial_msgs-srv:RequestParam-response instead.")))

(cl:ensure-generic-function 'ints-val :lambda-list '(m))
(cl:defmethod ints-val ((m <RequestParam-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rosserial_msgs-srv:ints-val is deprecated.  Use rosserial_msgs-srv:ints instead.")
  (ints m))

(cl:ensure-generic-function 'floats-val :lambda-list '(m))
(cl:defmethod floats-val ((m <RequestParam-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rosserial_msgs-srv:floats-val is deprecated.  Use rosserial_msgs-srv:floats instead.")
  (floats m))

(cl:ensure-generic-function 'strings-val :lambda-list '(m))
(cl:defmethod strings-val ((m <RequestParam-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rosserial_msgs-srv:strings-val is deprecated.  Use rosserial_msgs-srv:strings instead.")
  (strings m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RequestParam-response>) ostream)
  "Serializes a message object of type '<RequestParam-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'ints))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'ints))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'floats))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'floats))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'strings))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'strings))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RequestParam-response>) istream)
  "Deserializes a message object of type '<RequestParam-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'ints) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'ints)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296)))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'floats) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'floats)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'strings) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'strings)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RequestParam-response>)))
  "Returns string type for a service object of type '<RequestParam-response>"
  "rosserial_msgs/RequestParamResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RequestParam-response)))
  "Returns string type for a service object of type 'RequestParam-response"
  "rosserial_msgs/RequestParamResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RequestParam-response>)))
  "Returns md5sum for a message object of type '<RequestParam-response>"
  "d7a0c2be00c9fd03cc69f2863de9c4d9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RequestParam-response)))
  "Returns md5sum for a message object of type 'RequestParam-response"
  "d7a0c2be00c9fd03cc69f2863de9c4d9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RequestParam-response>)))
  "Returns full string definition for message of type '<RequestParam-response>"
  (cl:format cl:nil "~%int32[]   ints~%float32[] floats~%string[]  strings~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RequestParam-response)))
  "Returns full string definition for message of type 'RequestParam-response"
  (cl:format cl:nil "~%int32[]   ints~%float32[] floats~%string[]  strings~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RequestParam-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'ints) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'floats) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'strings) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RequestParam-response>))
  "Converts a ROS message object to a list"
  (cl:list 'RequestParam-response
    (cl:cons ':ints (ints msg))
    (cl:cons ':floats (floats msg))
    (cl:cons ':strings (strings msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'RequestParam)))
  'RequestParam-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'RequestParam)))
  'RequestParam-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RequestParam)))
  "Returns string type for a service object of type '<RequestParam>"
  "rosserial_msgs/RequestParam")
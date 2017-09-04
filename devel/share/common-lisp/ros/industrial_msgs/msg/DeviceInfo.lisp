; Auto-generated. Do not edit!


(cl:in-package industrial_msgs-msg)


;//! \htmlinclude DeviceInfo.msg.html

(cl:defclass <DeviceInfo> (roslisp-msg-protocol:ros-message)
  ((model
    :reader model
    :initarg :model
    :type cl:string
    :initform "")
   (serial_number
    :reader serial_number
    :initarg :serial_number
    :type cl:string
    :initform "")
   (hw_version
    :reader hw_version
    :initarg :hw_version
    :type cl:string
    :initform "")
   (sw_version
    :reader sw_version
    :initarg :sw_version
    :type cl:string
    :initform "")
   (address
    :reader address
    :initarg :address
    :type cl:string
    :initform ""))
)

(cl:defclass DeviceInfo (<DeviceInfo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DeviceInfo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DeviceInfo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name industrial_msgs-msg:<DeviceInfo> is deprecated: use industrial_msgs-msg:DeviceInfo instead.")))

(cl:ensure-generic-function 'model-val :lambda-list '(m))
(cl:defmethod model-val ((m <DeviceInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader industrial_msgs-msg:model-val is deprecated.  Use industrial_msgs-msg:model instead.")
  (model m))

(cl:ensure-generic-function 'serial_number-val :lambda-list '(m))
(cl:defmethod serial_number-val ((m <DeviceInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader industrial_msgs-msg:serial_number-val is deprecated.  Use industrial_msgs-msg:serial_number instead.")
  (serial_number m))

(cl:ensure-generic-function 'hw_version-val :lambda-list '(m))
(cl:defmethod hw_version-val ((m <DeviceInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader industrial_msgs-msg:hw_version-val is deprecated.  Use industrial_msgs-msg:hw_version instead.")
  (hw_version m))

(cl:ensure-generic-function 'sw_version-val :lambda-list '(m))
(cl:defmethod sw_version-val ((m <DeviceInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader industrial_msgs-msg:sw_version-val is deprecated.  Use industrial_msgs-msg:sw_version instead.")
  (sw_version m))

(cl:ensure-generic-function 'address-val :lambda-list '(m))
(cl:defmethod address-val ((m <DeviceInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader industrial_msgs-msg:address-val is deprecated.  Use industrial_msgs-msg:address instead.")
  (address m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DeviceInfo>) ostream)
  "Serializes a message object of type '<DeviceInfo>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'model))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'model))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'serial_number))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'serial_number))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'hw_version))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'hw_version))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'sw_version))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'sw_version))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'address))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'address))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DeviceInfo>) istream)
  "Deserializes a message object of type '<DeviceInfo>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'model) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'model) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'serial_number) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'serial_number) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'hw_version) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'hw_version) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'sw_version) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'sw_version) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'address) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'address) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DeviceInfo>)))
  "Returns string type for a message object of type '<DeviceInfo>"
  "industrial_msgs/DeviceInfo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DeviceInfo)))
  "Returns string type for a message object of type 'DeviceInfo"
  "industrial_msgs/DeviceInfo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DeviceInfo>)))
  "Returns md5sum for a message object of type '<DeviceInfo>"
  "373ed7fa0fac92d443be9cd5198e80f0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DeviceInfo)))
  "Returns md5sum for a message object of type 'DeviceInfo"
  "373ed7fa0fac92d443be9cd5198e80f0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DeviceInfo>)))
  "Returns full string definition for message of type '<DeviceInfo>"
  (cl:format cl:nil "# Device info captures device agnostic information about a piece of hardware.~%# This message is meant as a generic as possible.  Items that don't apply should~%# be left blank.  This message is not meant to replace diagnostic messages, but~%# rather provide a standard service message that can be used to populate standard~%# components (like a GUI for example)~%~%string model~%string serial_number~%string hw_version~%string sw_version~%string address~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DeviceInfo)))
  "Returns full string definition for message of type 'DeviceInfo"
  (cl:format cl:nil "# Device info captures device agnostic information about a piece of hardware.~%# This message is meant as a generic as possible.  Items that don't apply should~%# be left blank.  This message is not meant to replace diagnostic messages, but~%# rather provide a standard service message that can be used to populate standard~%# components (like a GUI for example)~%~%string model~%string serial_number~%string hw_version~%string sw_version~%string address~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DeviceInfo>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'model))
     4 (cl:length (cl:slot-value msg 'serial_number))
     4 (cl:length (cl:slot-value msg 'hw_version))
     4 (cl:length (cl:slot-value msg 'sw_version))
     4 (cl:length (cl:slot-value msg 'address))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DeviceInfo>))
  "Converts a ROS message object to a list"
  (cl:list 'DeviceInfo
    (cl:cons ':model (model msg))
    (cl:cons ':serial_number (serial_number msg))
    (cl:cons ':hw_version (hw_version msg))
    (cl:cons ':sw_version (sw_version msg))
    (cl:cons ':address (address msg))
))

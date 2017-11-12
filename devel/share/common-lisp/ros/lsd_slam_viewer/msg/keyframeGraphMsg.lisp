; Auto-generated. Do not edit!


(cl:in-package lsd_slam_viewer-msg)


;//! \htmlinclude keyframeGraphMsg.msg.html

(cl:defclass <keyframeGraphMsg> (roslisp-msg-protocol:ros-message)
  ((numFrames
    :reader numFrames
    :initarg :numFrames
    :type cl:integer
    :initform 0)
   (frameData
    :reader frameData
    :initarg :frameData
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0))
   (numConstraints
    :reader numConstraints
    :initarg :numConstraints
    :type cl:integer
    :initform 0)
   (constraintsData
    :reader constraintsData
    :initarg :constraintsData
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass keyframeGraphMsg (<keyframeGraphMsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <keyframeGraphMsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'keyframeGraphMsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name lsd_slam_viewer-msg:<keyframeGraphMsg> is deprecated: use lsd_slam_viewer-msg:keyframeGraphMsg instead.")))

(cl:ensure-generic-function 'numFrames-val :lambda-list '(m))
(cl:defmethod numFrames-val ((m <keyframeGraphMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lsd_slam_viewer-msg:numFrames-val is deprecated.  Use lsd_slam_viewer-msg:numFrames instead.")
  (numFrames m))

(cl:ensure-generic-function 'frameData-val :lambda-list '(m))
(cl:defmethod frameData-val ((m <keyframeGraphMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lsd_slam_viewer-msg:frameData-val is deprecated.  Use lsd_slam_viewer-msg:frameData instead.")
  (frameData m))

(cl:ensure-generic-function 'numConstraints-val :lambda-list '(m))
(cl:defmethod numConstraints-val ((m <keyframeGraphMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lsd_slam_viewer-msg:numConstraints-val is deprecated.  Use lsd_slam_viewer-msg:numConstraints instead.")
  (numConstraints m))

(cl:ensure-generic-function 'constraintsData-val :lambda-list '(m))
(cl:defmethod constraintsData-val ((m <keyframeGraphMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lsd_slam_viewer-msg:constraintsData-val is deprecated.  Use lsd_slam_viewer-msg:constraintsData instead.")
  (constraintsData m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <keyframeGraphMsg>) ostream)
  "Serializes a message object of type '<keyframeGraphMsg>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'numFrames)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'numFrames)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'numFrames)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'numFrames)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'frameData))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'frameData))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'numConstraints)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'numConstraints)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'numConstraints)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'numConstraints)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'constraintsData))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'constraintsData))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <keyframeGraphMsg>) istream)
  "Deserializes a message object of type '<keyframeGraphMsg>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'numFrames)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'numFrames)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'numFrames)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'numFrames)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'frameData) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'frameData)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'numConstraints)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'numConstraints)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'numConstraints)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'numConstraints)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'constraintsData) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'constraintsData)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<keyframeGraphMsg>)))
  "Returns string type for a message object of type '<keyframeGraphMsg>"
  "lsd_slam_viewer/keyframeGraphMsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'keyframeGraphMsg)))
  "Returns string type for a message object of type 'keyframeGraphMsg"
  "lsd_slam_viewer/keyframeGraphMsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<keyframeGraphMsg>)))
  "Returns md5sum for a message object of type '<keyframeGraphMsg>"
  "d23a8a86773b54db7399debf884d0c9e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'keyframeGraphMsg)))
  "Returns md5sum for a message object of type 'keyframeGraphMsg"
  "d23a8a86773b54db7399debf884d0c9e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<keyframeGraphMsg>)))
  "Returns full string definition for message of type '<keyframeGraphMsg>"
  (cl:format cl:nil "# data as serialization of sim(3)'s: (int id, float[7] camToWorld)~%uint32 numFrames~%uint8[] frameData~%~%~%# constraints (int from, int to, float err)~%uint32 numConstraints~%uint8[] constraintsData~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'keyframeGraphMsg)))
  "Returns full string definition for message of type 'keyframeGraphMsg"
  (cl:format cl:nil "# data as serialization of sim(3)'s: (int id, float[7] camToWorld)~%uint32 numFrames~%uint8[] frameData~%~%~%# constraints (int from, int to, float err)~%uint32 numConstraints~%uint8[] constraintsData~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <keyframeGraphMsg>))
  (cl:+ 0
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'frameData) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'constraintsData) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <keyframeGraphMsg>))
  "Converts a ROS message object to a list"
  (cl:list 'keyframeGraphMsg
    (cl:cons ':numFrames (numFrames msg))
    (cl:cons ':frameData (frameData msg))
    (cl:cons ':numConstraints (numConstraints msg))
    (cl:cons ':constraintsData (constraintsData msg))
))

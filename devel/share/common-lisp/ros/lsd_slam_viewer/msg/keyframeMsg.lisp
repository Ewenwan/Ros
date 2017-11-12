; Auto-generated. Do not edit!


(cl:in-package lsd_slam_viewer-msg)


;//! \htmlinclude keyframeMsg.msg.html

(cl:defclass <keyframeMsg> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:integer
    :initform 0)
   (time
    :reader time
    :initarg :time
    :type cl:float
    :initform 0.0)
   (isKeyframe
    :reader isKeyframe
    :initarg :isKeyframe
    :type cl:boolean
    :initform cl:nil)
   (camToWorld
    :reader camToWorld
    :initarg :camToWorld
    :type (cl:vector cl:float)
   :initform (cl:make-array 7 :element-type 'cl:float :initial-element 0.0))
   (fx
    :reader fx
    :initarg :fx
    :type cl:float
    :initform 0.0)
   (fy
    :reader fy
    :initarg :fy
    :type cl:float
    :initform 0.0)
   (cx
    :reader cx
    :initarg :cx
    :type cl:float
    :initform 0.0)
   (cy
    :reader cy
    :initarg :cy
    :type cl:float
    :initform 0.0)
   (height
    :reader height
    :initarg :height
    :type cl:integer
    :initform 0)
   (width
    :reader width
    :initarg :width
    :type cl:integer
    :initform 0)
   (pointcloud
    :reader pointcloud
    :initarg :pointcloud
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass keyframeMsg (<keyframeMsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <keyframeMsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'keyframeMsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name lsd_slam_viewer-msg:<keyframeMsg> is deprecated: use lsd_slam_viewer-msg:keyframeMsg instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <keyframeMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lsd_slam_viewer-msg:id-val is deprecated.  Use lsd_slam_viewer-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'time-val :lambda-list '(m))
(cl:defmethod time-val ((m <keyframeMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lsd_slam_viewer-msg:time-val is deprecated.  Use lsd_slam_viewer-msg:time instead.")
  (time m))

(cl:ensure-generic-function 'isKeyframe-val :lambda-list '(m))
(cl:defmethod isKeyframe-val ((m <keyframeMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lsd_slam_viewer-msg:isKeyframe-val is deprecated.  Use lsd_slam_viewer-msg:isKeyframe instead.")
  (isKeyframe m))

(cl:ensure-generic-function 'camToWorld-val :lambda-list '(m))
(cl:defmethod camToWorld-val ((m <keyframeMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lsd_slam_viewer-msg:camToWorld-val is deprecated.  Use lsd_slam_viewer-msg:camToWorld instead.")
  (camToWorld m))

(cl:ensure-generic-function 'fx-val :lambda-list '(m))
(cl:defmethod fx-val ((m <keyframeMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lsd_slam_viewer-msg:fx-val is deprecated.  Use lsd_slam_viewer-msg:fx instead.")
  (fx m))

(cl:ensure-generic-function 'fy-val :lambda-list '(m))
(cl:defmethod fy-val ((m <keyframeMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lsd_slam_viewer-msg:fy-val is deprecated.  Use lsd_slam_viewer-msg:fy instead.")
  (fy m))

(cl:ensure-generic-function 'cx-val :lambda-list '(m))
(cl:defmethod cx-val ((m <keyframeMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lsd_slam_viewer-msg:cx-val is deprecated.  Use lsd_slam_viewer-msg:cx instead.")
  (cx m))

(cl:ensure-generic-function 'cy-val :lambda-list '(m))
(cl:defmethod cy-val ((m <keyframeMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lsd_slam_viewer-msg:cy-val is deprecated.  Use lsd_slam_viewer-msg:cy instead.")
  (cy m))

(cl:ensure-generic-function 'height-val :lambda-list '(m))
(cl:defmethod height-val ((m <keyframeMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lsd_slam_viewer-msg:height-val is deprecated.  Use lsd_slam_viewer-msg:height instead.")
  (height m))

(cl:ensure-generic-function 'width-val :lambda-list '(m))
(cl:defmethod width-val ((m <keyframeMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lsd_slam_viewer-msg:width-val is deprecated.  Use lsd_slam_viewer-msg:width instead.")
  (width m))

(cl:ensure-generic-function 'pointcloud-val :lambda-list '(m))
(cl:defmethod pointcloud-val ((m <keyframeMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lsd_slam_viewer-msg:pointcloud-val is deprecated.  Use lsd_slam_viewer-msg:pointcloud instead.")
  (pointcloud m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <keyframeMsg>) ostream)
  "Serializes a message object of type '<keyframeMsg>"
  (cl:let* ((signed (cl:slot-value msg 'id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'time))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'isKeyframe) 1 0)) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'camToWorld))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'fx))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'fy))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'cx))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'cy))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'height)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'height)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'height)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'height)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'width)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'pointcloud))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'pointcloud))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <keyframeMsg>) istream)
  "Deserializes a message object of type '<keyframeMsg>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'time) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:slot-value msg 'isKeyframe) (cl:not (cl:zerop (cl:read-byte istream))))
  (cl:setf (cl:slot-value msg 'camToWorld) (cl:make-array 7))
  (cl:let ((vals (cl:slot-value msg 'camToWorld)))
    (cl:dotimes (i 7)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'fx) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'fy) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cx) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cy) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'height)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'height)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'height)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'height)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'width)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'pointcloud) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'pointcloud)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<keyframeMsg>)))
  "Returns string type for a message object of type '<keyframeMsg>"
  "lsd_slam_viewer/keyframeMsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'keyframeMsg)))
  "Returns string type for a message object of type 'keyframeMsg"
  "lsd_slam_viewer/keyframeMsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<keyframeMsg>)))
  "Returns md5sum for a message object of type '<keyframeMsg>"
  "42d4108dbb7d0e5d166eb68dd4054826")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'keyframeMsg)))
  "Returns md5sum for a message object of type 'keyframeMsg"
  "42d4108dbb7d0e5d166eb68dd4054826")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<keyframeMsg>)))
  "Returns full string definition for message of type '<keyframeMsg>"
  (cl:format cl:nil "int32 id~%float64 time~%bool isKeyframe~%~%# camToWorld as serialization of sophus sim(3).~%# may change with keyframeGraph-updates.~%float32[7] camToWorld ~%~%~%# camera parameter (fx fy cx cy), width, height~%# will never change, but required for display.~%float32 fx~%float32 fy~%float32 cx~%float32 cy~%uint32 height~%uint32 width~%~%~%# data as InputPointDense (float idepth, float idepth_var, uchar color[4]), width x height~%# may be empty, in that case no associated pointcloud is ever shown.~%uint8[] pointcloud~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'keyframeMsg)))
  "Returns full string definition for message of type 'keyframeMsg"
  (cl:format cl:nil "int32 id~%float64 time~%bool isKeyframe~%~%# camToWorld as serialization of sophus sim(3).~%# may change with keyframeGraph-updates.~%float32[7] camToWorld ~%~%~%# camera parameter (fx fy cx cy), width, height~%# will never change, but required for display.~%float32 fx~%float32 fy~%float32 cx~%float32 cy~%uint32 height~%uint32 width~%~%~%# data as InputPointDense (float idepth, float idepth_var, uchar color[4]), width x height~%# may be empty, in that case no associated pointcloud is ever shown.~%uint8[] pointcloud~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <keyframeMsg>))
  (cl:+ 0
     4
     8
     1
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'camToWorld) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4
     4
     4
     4
     4
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'pointcloud) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <keyframeMsg>))
  "Converts a ROS message object to a list"
  (cl:list 'keyframeMsg
    (cl:cons ':id (id msg))
    (cl:cons ':time (time msg))
    (cl:cons ':isKeyframe (isKeyframe msg))
    (cl:cons ':camToWorld (camToWorld msg))
    (cl:cons ':fx (fx msg))
    (cl:cons ':fy (fy msg))
    (cl:cons ':cx (cx msg))
    (cl:cons ':cy (cy msg))
    (cl:cons ':height (height msg))
    (cl:cons ':width (width msg))
    (cl:cons ':pointcloud (pointcloud msg))
))

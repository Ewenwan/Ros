; Auto-generated. Do not edit!


(cl:in-package face_recognition-msg)


;//! \htmlinclude FaceRecognitionResult.msg.html

(cl:defclass <FaceRecognitionResult> (roslisp-msg-protocol:ros-message)
  ((order_id
    :reader order_id
    :initarg :order_id
    :type cl:fixnum
    :initform 0)
   (names
    :reader names
    :initarg :names
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element ""))
   (confidence
    :reader confidence
    :initarg :confidence
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass FaceRecognitionResult (<FaceRecognitionResult>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FaceRecognitionResult>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FaceRecognitionResult)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name face_recognition-msg:<FaceRecognitionResult> is deprecated: use face_recognition-msg:FaceRecognitionResult instead.")))

(cl:ensure-generic-function 'order_id-val :lambda-list '(m))
(cl:defmethod order_id-val ((m <FaceRecognitionResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader face_recognition-msg:order_id-val is deprecated.  Use face_recognition-msg:order_id instead.")
  (order_id m))

(cl:ensure-generic-function 'names-val :lambda-list '(m))
(cl:defmethod names-val ((m <FaceRecognitionResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader face_recognition-msg:names-val is deprecated.  Use face_recognition-msg:names instead.")
  (names m))

(cl:ensure-generic-function 'confidence-val :lambda-list '(m))
(cl:defmethod confidence-val ((m <FaceRecognitionResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader face_recognition-msg:confidence-val is deprecated.  Use face_recognition-msg:confidence instead.")
  (confidence m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FaceRecognitionResult>) ostream)
  "Serializes a message object of type '<FaceRecognitionResult>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'order_id)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'names))))
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
   (cl:slot-value msg 'names))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'confidence))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'confidence))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FaceRecognitionResult>) istream)
  "Deserializes a message object of type '<FaceRecognitionResult>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'order_id)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'names) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'names)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'confidence) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'confidence)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FaceRecognitionResult>)))
  "Returns string type for a message object of type '<FaceRecognitionResult>"
  "face_recognition/FaceRecognitionResult")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FaceRecognitionResult)))
  "Returns string type for a message object of type 'FaceRecognitionResult"
  "face_recognition/FaceRecognitionResult")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FaceRecognitionResult>)))
  "Returns md5sum for a message object of type '<FaceRecognitionResult>"
  "1354b7478703dcb5d9e033b8bdfafb3b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FaceRecognitionResult)))
  "Returns md5sum for a message object of type 'FaceRecognitionResult"
  "1354b7478703dcb5d9e033b8bdfafb3b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FaceRecognitionResult>)))
  "Returns full string definition for message of type '<FaceRecognitionResult>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#result definition~%uint8   order_id~%string[]  names~%float32[] confidence~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FaceRecognitionResult)))
  "Returns full string definition for message of type 'FaceRecognitionResult"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#result definition~%uint8   order_id~%string[]  names~%float32[] confidence~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FaceRecognitionResult>))
  (cl:+ 0
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'names) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'confidence) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FaceRecognitionResult>))
  "Converts a ROS message object to a list"
  (cl:list 'FaceRecognitionResult
    (cl:cons ':order_id (order_id msg))
    (cl:cons ':names (names msg))
    (cl:cons ':confidence (confidence msg))
))

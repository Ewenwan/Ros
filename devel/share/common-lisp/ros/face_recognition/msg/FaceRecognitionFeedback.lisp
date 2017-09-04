; Auto-generated. Do not edit!


(cl:in-package face_recognition-msg)


;//! \htmlinclude FaceRecognitionFeedback.msg.html

(cl:defclass <FaceRecognitionFeedback> (roslisp-msg-protocol:ros-message)
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

(cl:defclass FaceRecognitionFeedback (<FaceRecognitionFeedback>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FaceRecognitionFeedback>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FaceRecognitionFeedback)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name face_recognition-msg:<FaceRecognitionFeedback> is deprecated: use face_recognition-msg:FaceRecognitionFeedback instead.")))

(cl:ensure-generic-function 'order_id-val :lambda-list '(m))
(cl:defmethod order_id-val ((m <FaceRecognitionFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader face_recognition-msg:order_id-val is deprecated.  Use face_recognition-msg:order_id instead.")
  (order_id m))

(cl:ensure-generic-function 'names-val :lambda-list '(m))
(cl:defmethod names-val ((m <FaceRecognitionFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader face_recognition-msg:names-val is deprecated.  Use face_recognition-msg:names instead.")
  (names m))

(cl:ensure-generic-function 'confidence-val :lambda-list '(m))
(cl:defmethod confidence-val ((m <FaceRecognitionFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader face_recognition-msg:confidence-val is deprecated.  Use face_recognition-msg:confidence instead.")
  (confidence m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FaceRecognitionFeedback>) ostream)
  "Serializes a message object of type '<FaceRecognitionFeedback>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FaceRecognitionFeedback>) istream)
  "Deserializes a message object of type '<FaceRecognitionFeedback>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FaceRecognitionFeedback>)))
  "Returns string type for a message object of type '<FaceRecognitionFeedback>"
  "face_recognition/FaceRecognitionFeedback")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FaceRecognitionFeedback)))
  "Returns string type for a message object of type 'FaceRecognitionFeedback"
  "face_recognition/FaceRecognitionFeedback")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FaceRecognitionFeedback>)))
  "Returns md5sum for a message object of type '<FaceRecognitionFeedback>"
  "1354b7478703dcb5d9e033b8bdfafb3b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FaceRecognitionFeedback)))
  "Returns md5sum for a message object of type 'FaceRecognitionFeedback"
  "1354b7478703dcb5d9e033b8bdfafb3b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FaceRecognitionFeedback>)))
  "Returns full string definition for message of type '<FaceRecognitionFeedback>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#feedback~%uint8   order_id~%string[]  names~%float32[] confidence~%~%~%~%~%~%~%################# order_id~%#order_id = 0: recognize_once~%#order_id = 1: recognize_continuous~%#order_id = 2  add_face_images~%#order_id = 3  (re)train~%#order_id = 4  exit~%~%################ order_argument~%# for the add_face_images goal, the order_argument specifies the name of the person for which training images are acquired from the camera.~%~%################ name and confidence in feedback and result~%# for Order_id = 0 or 1, name and confidence are the name and confidence of the recognized person in the video stream.~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FaceRecognitionFeedback)))
  "Returns full string definition for message of type 'FaceRecognitionFeedback"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#feedback~%uint8   order_id~%string[]  names~%float32[] confidence~%~%~%~%~%~%~%################# order_id~%#order_id = 0: recognize_once~%#order_id = 1: recognize_continuous~%#order_id = 2  add_face_images~%#order_id = 3  (re)train~%#order_id = 4  exit~%~%################ order_argument~%# for the add_face_images goal, the order_argument specifies the name of the person for which training images are acquired from the camera.~%~%################ name and confidence in feedback and result~%# for Order_id = 0 or 1, name and confidence are the name and confidence of the recognized person in the video stream.~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FaceRecognitionFeedback>))
  (cl:+ 0
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'names) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'confidence) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FaceRecognitionFeedback>))
  "Converts a ROS message object to a list"
  (cl:list 'FaceRecognitionFeedback
    (cl:cons ':order_id (order_id msg))
    (cl:cons ':names (names msg))
    (cl:cons ':confidence (confidence msg))
))

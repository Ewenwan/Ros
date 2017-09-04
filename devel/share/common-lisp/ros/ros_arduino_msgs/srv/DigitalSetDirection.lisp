; Auto-generated. Do not edit!


(cl:in-package ros_arduino_msgs-srv)


;//! \htmlinclude DigitalSetDirection-request.msg.html

(cl:defclass <DigitalSetDirection-request> (roslisp-msg-protocol:ros-message)
  ((pin
    :reader pin
    :initarg :pin
    :type cl:fixnum
    :initform 0)
   (direction
    :reader direction
    :initarg :direction
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass DigitalSetDirection-request (<DigitalSetDirection-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DigitalSetDirection-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DigitalSetDirection-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ros_arduino_msgs-srv:<DigitalSetDirection-request> is deprecated: use ros_arduino_msgs-srv:DigitalSetDirection-request instead.")))

(cl:ensure-generic-function 'pin-val :lambda-list '(m))
(cl:defmethod pin-val ((m <DigitalSetDirection-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_arduino_msgs-srv:pin-val is deprecated.  Use ros_arduino_msgs-srv:pin instead.")
  (pin m))

(cl:ensure-generic-function 'direction-val :lambda-list '(m))
(cl:defmethod direction-val ((m <DigitalSetDirection-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_arduino_msgs-srv:direction-val is deprecated.  Use ros_arduino_msgs-srv:direction instead.")
  (direction m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DigitalSetDirection-request>) ostream)
  "Serializes a message object of type '<DigitalSetDirection-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'pin)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'direction) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DigitalSetDirection-request>) istream)
  "Deserializes a message object of type '<DigitalSetDirection-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'pin)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'direction) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DigitalSetDirection-request>)))
  "Returns string type for a service object of type '<DigitalSetDirection-request>"
  "ros_arduino_msgs/DigitalSetDirectionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DigitalSetDirection-request)))
  "Returns string type for a service object of type 'DigitalSetDirection-request"
  "ros_arduino_msgs/DigitalSetDirectionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DigitalSetDirection-request>)))
  "Returns md5sum for a message object of type '<DigitalSetDirection-request>"
  "b10eff5e5e7e4623e1ee840cec92b372")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DigitalSetDirection-request)))
  "Returns md5sum for a message object of type 'DigitalSetDirection-request"
  "b10eff5e5e7e4623e1ee840cec92b372")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DigitalSetDirection-request>)))
  "Returns full string definition for message of type '<DigitalSetDirection-request>"
  (cl:format cl:nil "uint8 pin~%bool direction~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DigitalSetDirection-request)))
  "Returns full string definition for message of type 'DigitalSetDirection-request"
  (cl:format cl:nil "uint8 pin~%bool direction~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DigitalSetDirection-request>))
  (cl:+ 0
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DigitalSetDirection-request>))
  "Converts a ROS message object to a list"
  (cl:list 'DigitalSetDirection-request
    (cl:cons ':pin (pin msg))
    (cl:cons ':direction (direction msg))
))
;//! \htmlinclude DigitalSetDirection-response.msg.html

(cl:defclass <DigitalSetDirection-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass DigitalSetDirection-response (<DigitalSetDirection-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DigitalSetDirection-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DigitalSetDirection-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ros_arduino_msgs-srv:<DigitalSetDirection-response> is deprecated: use ros_arduino_msgs-srv:DigitalSetDirection-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DigitalSetDirection-response>) ostream)
  "Serializes a message object of type '<DigitalSetDirection-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DigitalSetDirection-response>) istream)
  "Deserializes a message object of type '<DigitalSetDirection-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DigitalSetDirection-response>)))
  "Returns string type for a service object of type '<DigitalSetDirection-response>"
  "ros_arduino_msgs/DigitalSetDirectionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DigitalSetDirection-response)))
  "Returns string type for a service object of type 'DigitalSetDirection-response"
  "ros_arduino_msgs/DigitalSetDirectionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DigitalSetDirection-response>)))
  "Returns md5sum for a message object of type '<DigitalSetDirection-response>"
  "b10eff5e5e7e4623e1ee840cec92b372")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DigitalSetDirection-response)))
  "Returns md5sum for a message object of type 'DigitalSetDirection-response"
  "b10eff5e5e7e4623e1ee840cec92b372")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DigitalSetDirection-response>)))
  "Returns full string definition for message of type '<DigitalSetDirection-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DigitalSetDirection-response)))
  "Returns full string definition for message of type 'DigitalSetDirection-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DigitalSetDirection-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DigitalSetDirection-response>))
  "Converts a ROS message object to a list"
  (cl:list 'DigitalSetDirection-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'DigitalSetDirection)))
  'DigitalSetDirection-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'DigitalSetDirection)))
  'DigitalSetDirection-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DigitalSetDirection)))
  "Returns string type for a service object of type '<DigitalSetDirection>"
  "ros_arduino_msgs/DigitalSetDirection")
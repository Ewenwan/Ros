
(cl:in-package :asdf)

(defsystem "face_recognition-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :actionlib_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "FRClientGoal" :depends-on ("_package_FRClientGoal"))
    (:file "_package_FRClientGoal" :depends-on ("_package"))
    (:file "FaceRecognitionResult" :depends-on ("_package_FaceRecognitionResult"))
    (:file "_package_FaceRecognitionResult" :depends-on ("_package"))
    (:file "FaceRecognitionActionFeedback" :depends-on ("_package_FaceRecognitionActionFeedback"))
    (:file "_package_FaceRecognitionActionFeedback" :depends-on ("_package"))
    (:file "FaceRecognitionActionResult" :depends-on ("_package_FaceRecognitionActionResult"))
    (:file "_package_FaceRecognitionActionResult" :depends-on ("_package"))
    (:file "FaceRecognitionActionGoal" :depends-on ("_package_FaceRecognitionActionGoal"))
    (:file "_package_FaceRecognitionActionGoal" :depends-on ("_package"))
    (:file "FaceRecognitionFeedback" :depends-on ("_package_FaceRecognitionFeedback"))
    (:file "_package_FaceRecognitionFeedback" :depends-on ("_package"))
    (:file "FaceRecognitionAction" :depends-on ("_package_FaceRecognitionAction"))
    (:file "_package_FaceRecognitionAction" :depends-on ("_package"))
    (:file "FaceRecognitionGoal" :depends-on ("_package_FaceRecognitionGoal"))
    (:file "_package_FaceRecognitionGoal" :depends-on ("_package"))
  ))
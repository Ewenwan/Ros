
(cl:in-package :asdf)

(defsystem "find_object_2d-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "ObjectsStamped" :depends-on ("_package_ObjectsStamped"))
    (:file "_package_ObjectsStamped" :depends-on ("_package"))
  ))
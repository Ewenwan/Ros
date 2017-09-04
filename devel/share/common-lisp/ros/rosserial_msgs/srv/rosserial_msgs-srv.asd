
(cl:in-package :asdf)

(defsystem "rosserial_msgs-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "RequestParam" :depends-on ("_package_RequestParam"))
    (:file "_package_RequestParam" :depends-on ("_package"))
    (:file "RequestServiceInfo" :depends-on ("_package_RequestServiceInfo"))
    (:file "_package_RequestServiceInfo" :depends-on ("_package"))
    (:file "RequestMessageInfo" :depends-on ("_package_RequestMessageInfo"))
    (:file "_package_RequestMessageInfo" :depends-on ("_package"))
  ))
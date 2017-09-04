
(cl:in-package :asdf)

(defsystem "industrial_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "TriState" :depends-on ("_package_TriState"))
    (:file "_package_TriState" :depends-on ("_package"))
    (:file "DeviceInfo" :depends-on ("_package_DeviceInfo"))
    (:file "_package_DeviceInfo" :depends-on ("_package"))
    (:file "RobotStatus" :depends-on ("_package_RobotStatus"))
    (:file "_package_RobotStatus" :depends-on ("_package"))
    (:file "RobotMode" :depends-on ("_package_RobotMode"))
    (:file "_package_RobotMode" :depends-on ("_package"))
    (:file "ServiceReturnCode" :depends-on ("_package_ServiceReturnCode"))
    (:file "_package_ServiceReturnCode" :depends-on ("_package"))
    (:file "DebugLevel" :depends-on ("_package_DebugLevel"))
    (:file "_package_DebugLevel" :depends-on ("_package"))
  ))
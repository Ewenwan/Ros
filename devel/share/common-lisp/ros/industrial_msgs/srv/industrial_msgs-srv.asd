
(cl:in-package :asdf)

(defsystem "industrial_msgs-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :industrial_msgs-msg
               :trajectory_msgs-msg
)
  :components ((:file "_package")
    (:file "GetRobotInfo" :depends-on ("_package_GetRobotInfo"))
    (:file "_package_GetRobotInfo" :depends-on ("_package"))
    (:file "SetDrivePower" :depends-on ("_package_SetDrivePower"))
    (:file "_package_SetDrivePower" :depends-on ("_package"))
    (:file "CmdJointTrajectory" :depends-on ("_package_CmdJointTrajectory"))
    (:file "_package_CmdJointTrajectory" :depends-on ("_package"))
    (:file "SetRemoteLoggerLevel" :depends-on ("_package_SetRemoteLoggerLevel"))
    (:file "_package_SetRemoteLoggerLevel" :depends-on ("_package"))
    (:file "StartMotion" :depends-on ("_package_StartMotion"))
    (:file "_package_StartMotion" :depends-on ("_package"))
    (:file "StopMotion" :depends-on ("_package_StopMotion"))
    (:file "_package_StopMotion" :depends-on ("_package"))
  ))
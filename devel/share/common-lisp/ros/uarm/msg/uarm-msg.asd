
(cl:in-package :asdf)

(defsystem "uarm-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "CoordsWithTime" :depends-on ("_package_CoordsWithTime"))
    (:file "_package_CoordsWithTime" :depends-on ("_package"))
    (:file "Coords" :depends-on ("_package_Coords"))
    (:file "_package_Coords" :depends-on ("_package"))
    (:file "CoordsWithTS4" :depends-on ("_package_CoordsWithTS4"))
    (:file "_package_CoordsWithTS4" :depends-on ("_package"))
    (:file "Angles" :depends-on ("_package_Angles"))
    (:file "_package_Angles" :depends-on ("_package"))
  ))
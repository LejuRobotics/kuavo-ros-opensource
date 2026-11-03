
(cl:in-package :asdf)

(defsystem "hardware_node-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "horizontalMotionByStep" :depends-on ("_package_horizontalMotionByStep"))
    (:file "_package_horizontalMotionByStep" :depends-on ("_package"))
    (:file "robotHeadMotionData" :depends-on ("_package_robotHeadMotionData"))
    (:file "_package_robotHeadMotionData" :depends-on ("_package"))
  ))
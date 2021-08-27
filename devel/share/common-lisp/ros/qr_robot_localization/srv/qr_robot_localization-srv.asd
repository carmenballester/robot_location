
(cl:in-package :asdf)

(defsystem "qr_robot_localization-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "DetectingQR" :depends-on ("_package_DetectingQR"))
    (:file "_package_DetectingQR" :depends-on ("_package"))
    (:file "Location" :depends-on ("_package_Location"))
    (:file "_package_Location" :depends-on ("_package"))
  ))
;; Auto-generated. Do not edit!


(when (boundp 'qr_robot_localization::Location)
  (if (not (find-package "QR_ROBOT_LOCALIZATION"))
    (make-package "QR_ROBOT_LOCALIZATION"))
  (shadow 'Location (find-package "QR_ROBOT_LOCALIZATION")))
(unless (find-package "QR_ROBOT_LOCALIZATION::LOCATION")
  (make-package "QR_ROBOT_LOCALIZATION::LOCATION"))
(unless (find-package "QR_ROBOT_LOCALIZATION::LOCATIONREQUEST")
  (make-package "QR_ROBOT_LOCALIZATION::LOCATIONREQUEST"))
(unless (find-package "QR_ROBOT_LOCALIZATION::LOCATIONRESPONSE")
  (make-package "QR_ROBOT_LOCALIZATION::LOCATIONRESPONSE"))

(in-package "ROS")





(defclass qr_robot_localization::LocationRequest
  :super ros::object
  :slots ())

(defmethod qr_robot_localization::LocationRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass qr_robot_localization::LocationResponse
  :super ros::object
  :slots (_node ))

(defmethod qr_robot_localization::LocationResponse
  (:init
   (&key
    ((:node __node) "")
    )
   (send-super :init)
   (setq _node (string __node))
   self)
  (:node
   (&optional __node)
   (if __node (setq _node __node)) _node)
  (:serialization-length
   ()
   (+
    ;; string _node
    4 (length _node)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _node
       (write-long (length _node) s) (princ _node s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _node
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _node (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass qr_robot_localization::Location
  :super ros::object
  :slots ())

(setf (get qr_robot_localization::Location :md5sum-) "a94c40e70a4b82863e6e52ec16732447")
(setf (get qr_robot_localization::Location :datatype-) "qr_robot_localization/Location")
(setf (get qr_robot_localization::Location :request) qr_robot_localization::LocationRequest)
(setf (get qr_robot_localization::Location :response) qr_robot_localization::LocationResponse)

(defmethod qr_robot_localization::LocationRequest
  (:response () (instance qr_robot_localization::LocationResponse :init)))

(setf (get qr_robot_localization::LocationRequest :md5sum-) "a94c40e70a4b82863e6e52ec16732447")
(setf (get qr_robot_localization::LocationRequest :datatype-) "qr_robot_localization/LocationRequest")
(setf (get qr_robot_localization::LocationRequest :definition-)
      "
---
string node

")

(setf (get qr_robot_localization::LocationResponse :md5sum-) "a94c40e70a4b82863e6e52ec16732447")
(setf (get qr_robot_localization::LocationResponse :datatype-) "qr_robot_localization/LocationResponse")
(setf (get qr_robot_localization::LocationResponse :definition-)
      "
---
string node

")



(provide :qr_robot_localization/Location "a94c40e70a4b82863e6e52ec16732447")



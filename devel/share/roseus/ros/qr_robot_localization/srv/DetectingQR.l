;; Auto-generated. Do not edit!


(when (boundp 'qr_robot_localization::DetectingQR)
  (if (not (find-package "QR_ROBOT_LOCALIZATION"))
    (make-package "QR_ROBOT_LOCALIZATION"))
  (shadow 'DetectingQR (find-package "QR_ROBOT_LOCALIZATION")))
(unless (find-package "QR_ROBOT_LOCALIZATION::DETECTINGQR")
  (make-package "QR_ROBOT_LOCALIZATION::DETECTINGQR"))
(unless (find-package "QR_ROBOT_LOCALIZATION::DETECTINGQRREQUEST")
  (make-package "QR_ROBOT_LOCALIZATION::DETECTINGQRREQUEST"))
(unless (find-package "QR_ROBOT_LOCALIZATION::DETECTINGQRRESPONSE")
  (make-package "QR_ROBOT_LOCALIZATION::DETECTINGQRRESPONSE"))

(in-package "ROS")





(defclass qr_robot_localization::DetectingQRRequest
  :super ros::object
  :slots ())

(defmethod qr_robot_localization::DetectingQRRequest
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

(defclass qr_robot_localization::DetectingQRResponse
  :super ros::object
  :slots (_node ))

(defmethod qr_robot_localization::DetectingQRResponse
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

(defclass qr_robot_localization::DetectingQR
  :super ros::object
  :slots ())

(setf (get qr_robot_localization::DetectingQR :md5sum-) "a94c40e70a4b82863e6e52ec16732447")
(setf (get qr_robot_localization::DetectingQR :datatype-) "qr_robot_localization/DetectingQR")
(setf (get qr_robot_localization::DetectingQR :request) qr_robot_localization::DetectingQRRequest)
(setf (get qr_robot_localization::DetectingQR :response) qr_robot_localization::DetectingQRResponse)

(defmethod qr_robot_localization::DetectingQRRequest
  (:response () (instance qr_robot_localization::DetectingQRResponse :init)))

(setf (get qr_robot_localization::DetectingQRRequest :md5sum-) "a94c40e70a4b82863e6e52ec16732447")
(setf (get qr_robot_localization::DetectingQRRequest :datatype-) "qr_robot_localization/DetectingQRRequest")
(setf (get qr_robot_localization::DetectingQRRequest :definition-)
      "
---
string node

")

(setf (get qr_robot_localization::DetectingQRResponse :md5sum-) "a94c40e70a4b82863e6e52ec16732447")
(setf (get qr_robot_localization::DetectingQRResponse :datatype-) "qr_robot_localization/DetectingQRResponse")
(setf (get qr_robot_localization::DetectingQRResponse :definition-)
      "
---
string node

")



(provide :qr_robot_localization/DetectingQR "a94c40e70a4b82863e6e52ec16732447")



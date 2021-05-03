;; Auto-generated. Do not edit!


(when (boundp 'qr_robot_control::Dijkstra)
  (if (not (find-package "QR_ROBOT_CONTROL"))
    (make-package "QR_ROBOT_CONTROL"))
  (shadow 'Dijkstra (find-package "QR_ROBOT_CONTROL")))
(unless (find-package "QR_ROBOT_CONTROL::DIJKSTRA")
  (make-package "QR_ROBOT_CONTROL::DIJKSTRA"))
(unless (find-package "QR_ROBOT_CONTROL::DIJKSTRAREQUEST")
  (make-package "QR_ROBOT_CONTROL::DIJKSTRAREQUEST"))
(unless (find-package "QR_ROBOT_CONTROL::DIJKSTRARESPONSE")
  (make-package "QR_ROBOT_CONTROL::DIJKSTRARESPONSE"))

(in-package "ROS")





(defclass qr_robot_control::DijkstraRequest
  :super ros::object
  :slots ())

(defmethod qr_robot_control::DijkstraRequest
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

(defclass qr_robot_control::DijkstraResponse
  :super ros::object
  :slots (_node ))

(defmethod qr_robot_control::DijkstraResponse
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

(defclass qr_robot_control::Dijkstra
  :super ros::object
  :slots ())

(setf (get qr_robot_control::Dijkstra :md5sum-) "a94c40e70a4b82863e6e52ec16732447")
(setf (get qr_robot_control::Dijkstra :datatype-) "qr_robot_control/Dijkstra")
(setf (get qr_robot_control::Dijkstra :request) qr_robot_control::DijkstraRequest)
(setf (get qr_robot_control::Dijkstra :response) qr_robot_control::DijkstraResponse)

(defmethod qr_robot_control::DijkstraRequest
  (:response () (instance qr_robot_control::DijkstraResponse :init)))

(setf (get qr_robot_control::DijkstraRequest :md5sum-) "a94c40e70a4b82863e6e52ec16732447")
(setf (get qr_robot_control::DijkstraRequest :datatype-) "qr_robot_control/DijkstraRequest")
(setf (get qr_robot_control::DijkstraRequest :definition-)
      "
---
string node

")

(setf (get qr_robot_control::DijkstraResponse :md5sum-) "a94c40e70a4b82863e6e52ec16732447")
(setf (get qr_robot_control::DijkstraResponse :datatype-) "qr_robot_control/DijkstraResponse")
(setf (get qr_robot_control::DijkstraResponse :definition-)
      "
---
string node

")



(provide :qr_robot_control/Dijkstra "a94c40e70a4b82863e6e52ec16732447")



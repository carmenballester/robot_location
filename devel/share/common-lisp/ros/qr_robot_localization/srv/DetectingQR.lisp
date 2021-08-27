; Auto-generated. Do not edit!


(cl:in-package qr_robot_localization-srv)


;//! \htmlinclude DetectingQR-request.msg.html

(cl:defclass <DetectingQR-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass DetectingQR-request (<DetectingQR-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DetectingQR-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DetectingQR-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name qr_robot_localization-srv:<DetectingQR-request> is deprecated: use qr_robot_localization-srv:DetectingQR-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DetectingQR-request>) ostream)
  "Serializes a message object of type '<DetectingQR-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DetectingQR-request>) istream)
  "Deserializes a message object of type '<DetectingQR-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DetectingQR-request>)))
  "Returns string type for a service object of type '<DetectingQR-request>"
  "qr_robot_localization/DetectingQRRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DetectingQR-request)))
  "Returns string type for a service object of type 'DetectingQR-request"
  "qr_robot_localization/DetectingQRRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DetectingQR-request>)))
  "Returns md5sum for a message object of type '<DetectingQR-request>"
  "a94c40e70a4b82863e6e52ec16732447")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DetectingQR-request)))
  "Returns md5sum for a message object of type 'DetectingQR-request"
  "a94c40e70a4b82863e6e52ec16732447")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DetectingQR-request>)))
  "Returns full string definition for message of type '<DetectingQR-request>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DetectingQR-request)))
  "Returns full string definition for message of type 'DetectingQR-request"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DetectingQR-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DetectingQR-request>))
  "Converts a ROS message object to a list"
  (cl:list 'DetectingQR-request
))
;//! \htmlinclude DetectingQR-response.msg.html

(cl:defclass <DetectingQR-response> (roslisp-msg-protocol:ros-message)
  ((node
    :reader node
    :initarg :node
    :type cl:string
    :initform ""))
)

(cl:defclass DetectingQR-response (<DetectingQR-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DetectingQR-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DetectingQR-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name qr_robot_localization-srv:<DetectingQR-response> is deprecated: use qr_robot_localization-srv:DetectingQR-response instead.")))

(cl:ensure-generic-function 'node-val :lambda-list '(m))
(cl:defmethod node-val ((m <DetectingQR-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qr_robot_localization-srv:node-val is deprecated.  Use qr_robot_localization-srv:node instead.")
  (node m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DetectingQR-response>) ostream)
  "Serializes a message object of type '<DetectingQR-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'node))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'node))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DetectingQR-response>) istream)
  "Deserializes a message object of type '<DetectingQR-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'node) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'node) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DetectingQR-response>)))
  "Returns string type for a service object of type '<DetectingQR-response>"
  "qr_robot_localization/DetectingQRResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DetectingQR-response)))
  "Returns string type for a service object of type 'DetectingQR-response"
  "qr_robot_localization/DetectingQRResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DetectingQR-response>)))
  "Returns md5sum for a message object of type '<DetectingQR-response>"
  "a94c40e70a4b82863e6e52ec16732447")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DetectingQR-response)))
  "Returns md5sum for a message object of type 'DetectingQR-response"
  "a94c40e70a4b82863e6e52ec16732447")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DetectingQR-response>)))
  "Returns full string definition for message of type '<DetectingQR-response>"
  (cl:format cl:nil "string node~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DetectingQR-response)))
  "Returns full string definition for message of type 'DetectingQR-response"
  (cl:format cl:nil "string node~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DetectingQR-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'node))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DetectingQR-response>))
  "Converts a ROS message object to a list"
  (cl:list 'DetectingQR-response
    (cl:cons ':node (node msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'DetectingQR)))
  'DetectingQR-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'DetectingQR)))
  'DetectingQR-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DetectingQR)))
  "Returns string type for a service object of type '<DetectingQR>"
  "qr_robot_localization/DetectingQR")
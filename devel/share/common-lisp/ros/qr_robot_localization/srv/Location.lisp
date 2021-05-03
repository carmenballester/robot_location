; Auto-generated. Do not edit!


(cl:in-package qr_robot_localization-srv)


;//! \htmlinclude Location-request.msg.html

(cl:defclass <Location-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Location-request (<Location-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Location-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Location-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name qr_robot_localization-srv:<Location-request> is deprecated: use qr_robot_localization-srv:Location-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Location-request>) ostream)
  "Serializes a message object of type '<Location-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Location-request>) istream)
  "Deserializes a message object of type '<Location-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Location-request>)))
  "Returns string type for a service object of type '<Location-request>"
  "qr_robot_localization/LocationRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Location-request)))
  "Returns string type for a service object of type 'Location-request"
  "qr_robot_localization/LocationRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Location-request>)))
  "Returns md5sum for a message object of type '<Location-request>"
  "a94c40e70a4b82863e6e52ec16732447")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Location-request)))
  "Returns md5sum for a message object of type 'Location-request"
  "a94c40e70a4b82863e6e52ec16732447")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Location-request>)))
  "Returns full string definition for message of type '<Location-request>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Location-request)))
  "Returns full string definition for message of type 'Location-request"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Location-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Location-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Location-request
))
;//! \htmlinclude Location-response.msg.html

(cl:defclass <Location-response> (roslisp-msg-protocol:ros-message)
  ((node
    :reader node
    :initarg :node
    :type cl:string
    :initform ""))
)

(cl:defclass Location-response (<Location-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Location-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Location-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name qr_robot_localization-srv:<Location-response> is deprecated: use qr_robot_localization-srv:Location-response instead.")))

(cl:ensure-generic-function 'node-val :lambda-list '(m))
(cl:defmethod node-val ((m <Location-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qr_robot_localization-srv:node-val is deprecated.  Use qr_robot_localization-srv:node instead.")
  (node m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Location-response>) ostream)
  "Serializes a message object of type '<Location-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'node))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'node))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Location-response>) istream)
  "Deserializes a message object of type '<Location-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Location-response>)))
  "Returns string type for a service object of type '<Location-response>"
  "qr_robot_localization/LocationResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Location-response)))
  "Returns string type for a service object of type 'Location-response"
  "qr_robot_localization/LocationResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Location-response>)))
  "Returns md5sum for a message object of type '<Location-response>"
  "a94c40e70a4b82863e6e52ec16732447")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Location-response)))
  "Returns md5sum for a message object of type 'Location-response"
  "a94c40e70a4b82863e6e52ec16732447")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Location-response>)))
  "Returns full string definition for message of type '<Location-response>"
  (cl:format cl:nil "string node~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Location-response)))
  "Returns full string definition for message of type 'Location-response"
  (cl:format cl:nil "string node~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Location-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'node))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Location-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Location-response
    (cl:cons ':node (node msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Location)))
  'Location-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Location)))
  'Location-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Location)))
  "Returns string type for a service object of type '<Location>"
  "qr_robot_localization/Location")
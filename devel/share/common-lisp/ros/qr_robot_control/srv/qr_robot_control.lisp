; Auto-generated. Do not edit!


(cl:in-package qr_robot_control-srv)


;//! \htmlinclude qr_robot_control-request.msg.html

(cl:defclass <qr_robot_control-request> (roslisp-msg-protocol:ros-message)
  ((a
    :reader a
    :initarg :a
    :type cl:integer
    :initform 0)
   (b
    :reader b
    :initarg :b
    :type cl:integer
    :initform 0)
   (c
    :reader c
    :initarg :c
    :type cl:integer
    :initform 0))
)

(cl:defclass qr_robot_control-request (<qr_robot_control-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <qr_robot_control-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'qr_robot_control-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name qr_robot_control-srv:<qr_robot_control-request> is deprecated: use qr_robot_control-srv:qr_robot_control-request instead.")))

(cl:ensure-generic-function 'a-val :lambda-list '(m))
(cl:defmethod a-val ((m <qr_robot_control-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qr_robot_control-srv:a-val is deprecated.  Use qr_robot_control-srv:a instead.")
  (a m))

(cl:ensure-generic-function 'b-val :lambda-list '(m))
(cl:defmethod b-val ((m <qr_robot_control-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qr_robot_control-srv:b-val is deprecated.  Use qr_robot_control-srv:b instead.")
  (b m))

(cl:ensure-generic-function 'c-val :lambda-list '(m))
(cl:defmethod c-val ((m <qr_robot_control-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qr_robot_control-srv:c-val is deprecated.  Use qr_robot_control-srv:c instead.")
  (c m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <qr_robot_control-request>) ostream)
  "Serializes a message object of type '<qr_robot_control-request>"
  (cl:let* ((signed (cl:slot-value msg 'a)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'b)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'c)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <qr_robot_control-request>) istream)
  "Deserializes a message object of type '<qr_robot_control-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'a) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'b) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'c) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<qr_robot_control-request>)))
  "Returns string type for a service object of type '<qr_robot_control-request>"
  "qr_robot_control/qr_robot_controlRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'qr_robot_control-request)))
  "Returns string type for a service object of type 'qr_robot_control-request"
  "qr_robot_control/qr_robot_controlRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<qr_robot_control-request>)))
  "Returns md5sum for a message object of type '<qr_robot_control-request>"
  "c9caccfcada3b6a45d8528a23bccaf1a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'qr_robot_control-request)))
  "Returns md5sum for a message object of type 'qr_robot_control-request"
  "c9caccfcada3b6a45d8528a23bccaf1a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<qr_robot_control-request>)))
  "Returns full string definition for message of type '<qr_robot_control-request>"
  (cl:format cl:nil "int64 a~%int64 b~%int64 c~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'qr_robot_control-request)))
  "Returns full string definition for message of type 'qr_robot_control-request"
  (cl:format cl:nil "int64 a~%int64 b~%int64 c~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <qr_robot_control-request>))
  (cl:+ 0
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <qr_robot_control-request>))
  "Converts a ROS message object to a list"
  (cl:list 'qr_robot_control-request
    (cl:cons ':a (a msg))
    (cl:cons ':b (b msg))
    (cl:cons ':c (c msg))
))
;//! \htmlinclude qr_robot_control-response.msg.html

(cl:defclass <qr_robot_control-response> (roslisp-msg-protocol:ros-message)
  ((sum
    :reader sum
    :initarg :sum
    :type cl:integer
    :initform 0))
)

(cl:defclass qr_robot_control-response (<qr_robot_control-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <qr_robot_control-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'qr_robot_control-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name qr_robot_control-srv:<qr_robot_control-response> is deprecated: use qr_robot_control-srv:qr_robot_control-response instead.")))

(cl:ensure-generic-function 'sum-val :lambda-list '(m))
(cl:defmethod sum-val ((m <qr_robot_control-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qr_robot_control-srv:sum-val is deprecated.  Use qr_robot_control-srv:sum instead.")
  (sum m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <qr_robot_control-response>) ostream)
  "Serializes a message object of type '<qr_robot_control-response>"
  (cl:let* ((signed (cl:slot-value msg 'sum)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <qr_robot_control-response>) istream)
  "Deserializes a message object of type '<qr_robot_control-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'sum) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<qr_robot_control-response>)))
  "Returns string type for a service object of type '<qr_robot_control-response>"
  "qr_robot_control/qr_robot_controlResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'qr_robot_control-response)))
  "Returns string type for a service object of type 'qr_robot_control-response"
  "qr_robot_control/qr_robot_controlResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<qr_robot_control-response>)))
  "Returns md5sum for a message object of type '<qr_robot_control-response>"
  "c9caccfcada3b6a45d8528a23bccaf1a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'qr_robot_control-response)))
  "Returns md5sum for a message object of type 'qr_robot_control-response"
  "c9caccfcada3b6a45d8528a23bccaf1a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<qr_robot_control-response>)))
  "Returns full string definition for message of type '<qr_robot_control-response>"
  (cl:format cl:nil "int64 sum~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'qr_robot_control-response)))
  "Returns full string definition for message of type 'qr_robot_control-response"
  (cl:format cl:nil "int64 sum~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <qr_robot_control-response>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <qr_robot_control-response>))
  "Converts a ROS message object to a list"
  (cl:list 'qr_robot_control-response
    (cl:cons ':sum (sum msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'qr_robot_control)))
  'qr_robot_control-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'qr_robot_control)))
  'qr_robot_control-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'qr_robot_control)))
  "Returns string type for a service object of type '<qr_robot_control>"
  "qr_robot_control/qr_robot_control")
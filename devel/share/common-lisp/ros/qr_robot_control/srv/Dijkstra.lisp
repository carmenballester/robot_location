; Auto-generated. Do not edit!


(cl:in-package qr_robot_control-srv)


;//! \htmlinclude Dijkstra-request.msg.html

(cl:defclass <Dijkstra-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Dijkstra-request (<Dijkstra-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Dijkstra-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Dijkstra-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name qr_robot_control-srv:<Dijkstra-request> is deprecated: use qr_robot_control-srv:Dijkstra-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Dijkstra-request>) ostream)
  "Serializes a message object of type '<Dijkstra-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Dijkstra-request>) istream)
  "Deserializes a message object of type '<Dijkstra-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Dijkstra-request>)))
  "Returns string type for a service object of type '<Dijkstra-request>"
  "qr_robot_control/DijkstraRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Dijkstra-request)))
  "Returns string type for a service object of type 'Dijkstra-request"
  "qr_robot_control/DijkstraRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Dijkstra-request>)))
  "Returns md5sum for a message object of type '<Dijkstra-request>"
  "a94c40e70a4b82863e6e52ec16732447")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Dijkstra-request)))
  "Returns md5sum for a message object of type 'Dijkstra-request"
  "a94c40e70a4b82863e6e52ec16732447")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Dijkstra-request>)))
  "Returns full string definition for message of type '<Dijkstra-request>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Dijkstra-request)))
  "Returns full string definition for message of type 'Dijkstra-request"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Dijkstra-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Dijkstra-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Dijkstra-request
))
;//! \htmlinclude Dijkstra-response.msg.html

(cl:defclass <Dijkstra-response> (roslisp-msg-protocol:ros-message)
  ((node
    :reader node
    :initarg :node
    :type cl:string
    :initform ""))
)

(cl:defclass Dijkstra-response (<Dijkstra-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Dijkstra-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Dijkstra-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name qr_robot_control-srv:<Dijkstra-response> is deprecated: use qr_robot_control-srv:Dijkstra-response instead.")))

(cl:ensure-generic-function 'node-val :lambda-list '(m))
(cl:defmethod node-val ((m <Dijkstra-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qr_robot_control-srv:node-val is deprecated.  Use qr_robot_control-srv:node instead.")
  (node m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Dijkstra-response>) ostream)
  "Serializes a message object of type '<Dijkstra-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'node))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'node))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Dijkstra-response>) istream)
  "Deserializes a message object of type '<Dijkstra-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Dijkstra-response>)))
  "Returns string type for a service object of type '<Dijkstra-response>"
  "qr_robot_control/DijkstraResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Dijkstra-response)))
  "Returns string type for a service object of type 'Dijkstra-response"
  "qr_robot_control/DijkstraResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Dijkstra-response>)))
  "Returns md5sum for a message object of type '<Dijkstra-response>"
  "a94c40e70a4b82863e6e52ec16732447")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Dijkstra-response)))
  "Returns md5sum for a message object of type 'Dijkstra-response"
  "a94c40e70a4b82863e6e52ec16732447")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Dijkstra-response>)))
  "Returns full string definition for message of type '<Dijkstra-response>"
  (cl:format cl:nil "string node~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Dijkstra-response)))
  "Returns full string definition for message of type 'Dijkstra-response"
  (cl:format cl:nil "string node~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Dijkstra-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'node))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Dijkstra-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Dijkstra-response
    (cl:cons ':node (node msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Dijkstra)))
  'Dijkstra-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Dijkstra)))
  'Dijkstra-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Dijkstra)))
  "Returns string type for a service object of type '<Dijkstra>"
  "qr_robot_control/Dijkstra")
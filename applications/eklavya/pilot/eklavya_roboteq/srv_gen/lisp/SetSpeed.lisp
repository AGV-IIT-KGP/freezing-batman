; Auto-generated. Do not edit!


(cl:in-package eklavya_roboteq-srv)


;//! \htmlinclude SetSpeed-request.msg.html

(cl:defclass <SetSpeed-request> (roslisp-msg-protocol:ros-message)
  ((left_speed
    :reader left_speed
    :initarg :left_speed
    :type cl:integer
    :initform 0)
   (right_speed
    :reader right_speed
    :initarg :right_speed
    :type cl:integer
    :initform 0))
)

(cl:defclass SetSpeed-request (<SetSpeed-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetSpeed-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetSpeed-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name eklavya_roboteq-srv:<SetSpeed-request> is deprecated: use eklavya_roboteq-srv:SetSpeed-request instead.")))

(cl:ensure-generic-function 'left_speed-val :lambda-list '(m))
(cl:defmethod left_speed-val ((m <SetSpeed-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader eklavya_roboteq-srv:left_speed-val is deprecated.  Use eklavya_roboteq-srv:left_speed instead.")
  (left_speed m))

(cl:ensure-generic-function 'right_speed-val :lambda-list '(m))
(cl:defmethod right_speed-val ((m <SetSpeed-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader eklavya_roboteq-srv:right_speed-val is deprecated.  Use eklavya_roboteq-srv:right_speed instead.")
  (right_speed m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetSpeed-request>) ostream)
  "Serializes a message object of type '<SetSpeed-request>"
  (cl:let* ((signed (cl:slot-value msg 'left_speed)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'right_speed)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetSpeed-request>) istream)
  "Deserializes a message object of type '<SetSpeed-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'left_speed) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'right_speed) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetSpeed-request>)))
  "Returns string type for a service object of type '<SetSpeed-request>"
  "eklavya_roboteq/SetSpeedRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetSpeed-request)))
  "Returns string type for a service object of type 'SetSpeed-request"
  "eklavya_roboteq/SetSpeedRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetSpeed-request>)))
  "Returns md5sum for a message object of type '<SetSpeed-request>"
  "c9c458c5a00e8258568427405d7b4a58")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetSpeed-request)))
  "Returns md5sum for a message object of type 'SetSpeed-request"
  "c9c458c5a00e8258568427405d7b4a58")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetSpeed-request>)))
  "Returns full string definition for message of type '<SetSpeed-request>"
  (cl:format cl:nil "int64 left_speed~%int64 right_speed~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetSpeed-request)))
  "Returns full string definition for message of type 'SetSpeed-request"
  (cl:format cl:nil "int64 left_speed~%int64 right_speed~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetSpeed-request>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetSpeed-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetSpeed-request
    (cl:cons ':left_speed (left_speed msg))
    (cl:cons ':right_speed (right_speed msg))
))
;//! \htmlinclude SetSpeed-response.msg.html

(cl:defclass <SetSpeed-response> (roslisp-msg-protocol:ros-message)
  ((code
    :reader code
    :initarg :code
    :type cl:integer
    :initform 0))
)

(cl:defclass SetSpeed-response (<SetSpeed-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetSpeed-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetSpeed-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name eklavya_roboteq-srv:<SetSpeed-response> is deprecated: use eklavya_roboteq-srv:SetSpeed-response instead.")))

(cl:ensure-generic-function 'code-val :lambda-list '(m))
(cl:defmethod code-val ((m <SetSpeed-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader eklavya_roboteq-srv:code-val is deprecated.  Use eklavya_roboteq-srv:code instead.")
  (code m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetSpeed-response>) ostream)
  "Serializes a message object of type '<SetSpeed-response>"
  (cl:let* ((signed (cl:slot-value msg 'code)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetSpeed-response>) istream)
  "Deserializes a message object of type '<SetSpeed-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'code) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetSpeed-response>)))
  "Returns string type for a service object of type '<SetSpeed-response>"
  "eklavya_roboteq/SetSpeedResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetSpeed-response)))
  "Returns string type for a service object of type 'SetSpeed-response"
  "eklavya_roboteq/SetSpeedResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetSpeed-response>)))
  "Returns md5sum for a message object of type '<SetSpeed-response>"
  "c9c458c5a00e8258568427405d7b4a58")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetSpeed-response)))
  "Returns md5sum for a message object of type 'SetSpeed-response"
  "c9c458c5a00e8258568427405d7b4a58")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetSpeed-response>)))
  "Returns full string definition for message of type '<SetSpeed-response>"
  (cl:format cl:nil "int64 code~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetSpeed-response)))
  "Returns full string definition for message of type 'SetSpeed-response"
  (cl:format cl:nil "int64 code~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetSpeed-response>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetSpeed-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetSpeed-response
    (cl:cons ':code (code msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetSpeed)))
  'SetSpeed-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetSpeed)))
  'SetSpeed-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetSpeed)))
  "Returns string type for a service object of type '<SetSpeed>"
  "eklavya_roboteq/SetSpeed")
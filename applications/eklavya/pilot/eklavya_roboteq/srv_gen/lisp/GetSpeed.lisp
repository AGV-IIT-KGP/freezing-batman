; Auto-generated. Do not edit!


(cl:in-package eklavya_roboteq-srv)


;//! \htmlinclude GetSpeed-request.msg.html

(cl:defclass <GetSpeed-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetSpeed-request (<GetSpeed-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetSpeed-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetSpeed-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name eklavya_roboteq-srv:<GetSpeed-request> is deprecated: use eklavya_roboteq-srv:GetSpeed-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetSpeed-request>) ostream)
  "Serializes a message object of type '<GetSpeed-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetSpeed-request>) istream)
  "Deserializes a message object of type '<GetSpeed-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetSpeed-request>)))
  "Returns string type for a service object of type '<GetSpeed-request>"
  "eklavya_roboteq/GetSpeedRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetSpeed-request)))
  "Returns string type for a service object of type 'GetSpeed-request"
  "eklavya_roboteq/GetSpeedRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetSpeed-request>)))
  "Returns md5sum for a message object of type '<GetSpeed-request>"
  "0e0c05cffc768dcd7d6b1b4e756a56bd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetSpeed-request)))
  "Returns md5sum for a message object of type 'GetSpeed-request"
  "0e0c05cffc768dcd7d6b1b4e756a56bd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetSpeed-request>)))
  "Returns full string definition for message of type '<GetSpeed-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetSpeed-request)))
  "Returns full string definition for message of type 'GetSpeed-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetSpeed-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetSpeed-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetSpeed-request
))
;//! \htmlinclude GetSpeed-response.msg.html

(cl:defclass <GetSpeed-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass GetSpeed-response (<GetSpeed-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetSpeed-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetSpeed-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name eklavya_roboteq-srv:<GetSpeed-response> is deprecated: use eklavya_roboteq-srv:GetSpeed-response instead.")))

(cl:ensure-generic-function 'left_speed-val :lambda-list '(m))
(cl:defmethod left_speed-val ((m <GetSpeed-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader eklavya_roboteq-srv:left_speed-val is deprecated.  Use eklavya_roboteq-srv:left_speed instead.")
  (left_speed m))

(cl:ensure-generic-function 'right_speed-val :lambda-list '(m))
(cl:defmethod right_speed-val ((m <GetSpeed-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader eklavya_roboteq-srv:right_speed-val is deprecated.  Use eklavya_roboteq-srv:right_speed instead.")
  (right_speed m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetSpeed-response>) ostream)
  "Serializes a message object of type '<GetSpeed-response>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetSpeed-response>) istream)
  "Deserializes a message object of type '<GetSpeed-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetSpeed-response>)))
  "Returns string type for a service object of type '<GetSpeed-response>"
  "eklavya_roboteq/GetSpeedResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetSpeed-response)))
  "Returns string type for a service object of type 'GetSpeed-response"
  "eklavya_roboteq/GetSpeedResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetSpeed-response>)))
  "Returns md5sum for a message object of type '<GetSpeed-response>"
  "0e0c05cffc768dcd7d6b1b4e756a56bd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetSpeed-response)))
  "Returns md5sum for a message object of type 'GetSpeed-response"
  "0e0c05cffc768dcd7d6b1b4e756a56bd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetSpeed-response>)))
  "Returns full string definition for message of type '<GetSpeed-response>"
  (cl:format cl:nil "int64 left_speed~%int64 right_speed~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetSpeed-response)))
  "Returns full string definition for message of type 'GetSpeed-response"
  (cl:format cl:nil "int64 left_speed~%int64 right_speed~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetSpeed-response>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetSpeed-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetSpeed-response
    (cl:cons ':left_speed (left_speed msg))
    (cl:cons ':right_speed (right_speed msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetSpeed)))
  'GetSpeed-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetSpeed)))
  'GetSpeed-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetSpeed)))
  "Returns string type for a service object of type '<GetSpeed>"
  "eklavya_roboteq/GetSpeed")
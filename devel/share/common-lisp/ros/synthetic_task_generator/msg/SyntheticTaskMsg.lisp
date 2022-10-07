; Auto-generated. Do not edit!


(cl:in-package synthetic_task_generator-msg)


;//! \htmlinclude SyntheticTaskMsg.msg.html

(cl:defclass <SyntheticTaskMsg> (roslisp-msg-protocol:ros-message)
  ((instance
    :reader instance
    :initarg :instance
    :type cl:integer
    :initform 0)
   (value
    :reader value
    :initarg :value
    :type cl:float
    :initform 0.0))
)

(cl:defclass SyntheticTaskMsg (<SyntheticTaskMsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SyntheticTaskMsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SyntheticTaskMsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name synthetic_task_generator-msg:<SyntheticTaskMsg> is deprecated: use synthetic_task_generator-msg:SyntheticTaskMsg instead.")))

(cl:ensure-generic-function 'instance-val :lambda-list '(m))
(cl:defmethod instance-val ((m <SyntheticTaskMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader synthetic_task_generator-msg:instance-val is deprecated.  Use synthetic_task_generator-msg:instance instead.")
  (instance m))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <SyntheticTaskMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader synthetic_task_generator-msg:value-val is deprecated.  Use synthetic_task_generator-msg:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SyntheticTaskMsg>) ostream)
  "Serializes a message object of type '<SyntheticTaskMsg>"
  (cl:let* ((signed (cl:slot-value msg 'instance)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'value))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SyntheticTaskMsg>) istream)
  "Deserializes a message object of type '<SyntheticTaskMsg>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'instance) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'value) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SyntheticTaskMsg>)))
  "Returns string type for a message object of type '<SyntheticTaskMsg>"
  "synthetic_task_generator/SyntheticTaskMsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SyntheticTaskMsg)))
  "Returns string type for a message object of type 'SyntheticTaskMsg"
  "synthetic_task_generator/SyntheticTaskMsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SyntheticTaskMsg>)))
  "Returns md5sum for a message object of type '<SyntheticTaskMsg>"
  "cbfe4c158f332960f1182cd0ed832cbc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SyntheticTaskMsg)))
  "Returns md5sum for a message object of type 'SyntheticTaskMsg"
  "cbfe4c158f332960f1182cd0ed832cbc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SyntheticTaskMsg>)))
  "Returns full string definition for message of type '<SyntheticTaskMsg>"
  (cl:format cl:nil "int64 instance~%float64 value~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SyntheticTaskMsg)))
  "Returns full string definition for message of type 'SyntheticTaskMsg"
  (cl:format cl:nil "int64 instance~%float64 value~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SyntheticTaskMsg>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SyntheticTaskMsg>))
  "Converts a ROS message object to a list"
  (cl:list 'SyntheticTaskMsg
    (cl:cons ':instance (instance msg))
    (cl:cons ':value (value msg))
))

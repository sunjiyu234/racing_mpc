; Auto-generated. Do not edit!


(cl:in-package pred_msg-msg)


;//! \htmlinclude correct_data.msg.html

(cl:defclass <correct_data> (roslisp-msg-protocol:ros-message)
  ((index_data
    :reader index_data
    :initarg :index_data
    :type cl:integer
    :initform 0)
   (correct_v
    :reader correct_v
    :initarg :correct_v
    :type cl:float
    :initform 0.0)
   (correct_r
    :reader correct_r
    :initarg :correct_r
    :type cl:float
    :initform 0.0)
   (correct_beta
    :reader correct_beta
    :initarg :correct_beta
    :type cl:float
    :initform 0.0))
)

(cl:defclass correct_data (<correct_data>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <correct_data>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'correct_data)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pred_msg-msg:<correct_data> is deprecated: use pred_msg-msg:correct_data instead.")))

(cl:ensure-generic-function 'index_data-val :lambda-list '(m))
(cl:defmethod index_data-val ((m <correct_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pred_msg-msg:index_data-val is deprecated.  Use pred_msg-msg:index_data instead.")
  (index_data m))

(cl:ensure-generic-function 'correct_v-val :lambda-list '(m))
(cl:defmethod correct_v-val ((m <correct_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pred_msg-msg:correct_v-val is deprecated.  Use pred_msg-msg:correct_v instead.")
  (correct_v m))

(cl:ensure-generic-function 'correct_r-val :lambda-list '(m))
(cl:defmethod correct_r-val ((m <correct_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pred_msg-msg:correct_r-val is deprecated.  Use pred_msg-msg:correct_r instead.")
  (correct_r m))

(cl:ensure-generic-function 'correct_beta-val :lambda-list '(m))
(cl:defmethod correct_beta-val ((m <correct_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pred_msg-msg:correct_beta-val is deprecated.  Use pred_msg-msg:correct_beta instead.")
  (correct_beta m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <correct_data>) ostream)
  "Serializes a message object of type '<correct_data>"
  (cl:let* ((signed (cl:slot-value msg 'index_data)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'correct_v))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'correct_r))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'correct_beta))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <correct_data>) istream)
  "Deserializes a message object of type '<correct_data>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'index_data) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'correct_v) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'correct_r) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'correct_beta) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<correct_data>)))
  "Returns string type for a message object of type '<correct_data>"
  "pred_msg/correct_data")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'correct_data)))
  "Returns string type for a message object of type 'correct_data"
  "pred_msg/correct_data")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<correct_data>)))
  "Returns md5sum for a message object of type '<correct_data>"
  "41f364437c141998133be277ae3d31c7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'correct_data)))
  "Returns md5sum for a message object of type 'correct_data"
  "41f364437c141998133be277ae3d31c7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<correct_data>)))
  "Returns full string definition for message of type '<correct_data>"
  (cl:format cl:nil "int32 index_data~%float64 correct_v~%float64 correct_r~%float64 correct_beta~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'correct_data)))
  "Returns full string definition for message of type 'correct_data"
  (cl:format cl:nil "int32 index_data~%float64 correct_v~%float64 correct_r~%float64 correct_beta~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <correct_data>))
  (cl:+ 0
     4
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <correct_data>))
  "Converts a ROS message object to a list"
  (cl:list 'correct_data
    (cl:cons ':index_data (index_data msg))
    (cl:cons ':correct_v (correct_v msg))
    (cl:cons ':correct_r (correct_r msg))
    (cl:cons ':correct_beta (correct_beta msg))
))

; Auto-generated. Do not edit!


(cl:in-package pred_msg-msg)


;//! \htmlinclude state_data.msg.html

(cl:defclass <state_data> (roslisp-msg-protocol:ros-message)
  ((index_data
    :reader index_data
    :initarg :index_data
    :type cl:integer
    :initform 0)
   (input_v
    :reader input_v
    :initarg :input_v
    :type cl:float
    :initform 0.0)
   (input_r
    :reader input_r
    :initarg :input_r
    :type cl:float
    :initform 0.0)
   (input_beta
    :reader input_beta
    :initarg :input_beta
    :type cl:float
    :initform 0.0)
   (input_Tf
    :reader input_Tf
    :initarg :input_Tf
    :type cl:float
    :initform 0.0)
   (input_Tr
    :reader input_Tr
    :initarg :input_Tr
    :type cl:float
    :initform 0.0)
   (input_delta
    :reader input_delta
    :initarg :input_delta
    :type cl:float
    :initform 0.0)
   (input_error_v
    :reader input_error_v
    :initarg :input_error_v
    :type cl:float
    :initform 0.0)
   (input_error_r
    :reader input_error_r
    :initarg :input_error_r
    :type cl:float
    :initform 0.0)
   (input_error_beta
    :reader input_error_beta
    :initarg :input_error_beta
    :type cl:float
    :initform 0.0))
)

(cl:defclass state_data (<state_data>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <state_data>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'state_data)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pred_msg-msg:<state_data> is deprecated: use pred_msg-msg:state_data instead.")))

(cl:ensure-generic-function 'index_data-val :lambda-list '(m))
(cl:defmethod index_data-val ((m <state_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pred_msg-msg:index_data-val is deprecated.  Use pred_msg-msg:index_data instead.")
  (index_data m))

(cl:ensure-generic-function 'input_v-val :lambda-list '(m))
(cl:defmethod input_v-val ((m <state_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pred_msg-msg:input_v-val is deprecated.  Use pred_msg-msg:input_v instead.")
  (input_v m))

(cl:ensure-generic-function 'input_r-val :lambda-list '(m))
(cl:defmethod input_r-val ((m <state_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pred_msg-msg:input_r-val is deprecated.  Use pred_msg-msg:input_r instead.")
  (input_r m))

(cl:ensure-generic-function 'input_beta-val :lambda-list '(m))
(cl:defmethod input_beta-val ((m <state_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pred_msg-msg:input_beta-val is deprecated.  Use pred_msg-msg:input_beta instead.")
  (input_beta m))

(cl:ensure-generic-function 'input_Tf-val :lambda-list '(m))
(cl:defmethod input_Tf-val ((m <state_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pred_msg-msg:input_Tf-val is deprecated.  Use pred_msg-msg:input_Tf instead.")
  (input_Tf m))

(cl:ensure-generic-function 'input_Tr-val :lambda-list '(m))
(cl:defmethod input_Tr-val ((m <state_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pred_msg-msg:input_Tr-val is deprecated.  Use pred_msg-msg:input_Tr instead.")
  (input_Tr m))

(cl:ensure-generic-function 'input_delta-val :lambda-list '(m))
(cl:defmethod input_delta-val ((m <state_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pred_msg-msg:input_delta-val is deprecated.  Use pred_msg-msg:input_delta instead.")
  (input_delta m))

(cl:ensure-generic-function 'input_error_v-val :lambda-list '(m))
(cl:defmethod input_error_v-val ((m <state_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pred_msg-msg:input_error_v-val is deprecated.  Use pred_msg-msg:input_error_v instead.")
  (input_error_v m))

(cl:ensure-generic-function 'input_error_r-val :lambda-list '(m))
(cl:defmethod input_error_r-val ((m <state_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pred_msg-msg:input_error_r-val is deprecated.  Use pred_msg-msg:input_error_r instead.")
  (input_error_r m))

(cl:ensure-generic-function 'input_error_beta-val :lambda-list '(m))
(cl:defmethod input_error_beta-val ((m <state_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pred_msg-msg:input_error_beta-val is deprecated.  Use pred_msg-msg:input_error_beta instead.")
  (input_error_beta m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <state_data>) ostream)
  "Serializes a message object of type '<state_data>"
  (cl:let* ((signed (cl:slot-value msg 'index_data)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'input_v))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'input_r))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'input_beta))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'input_Tf))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'input_Tr))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'input_delta))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'input_error_v))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'input_error_r))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'input_error_beta))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <state_data>) istream)
  "Deserializes a message object of type '<state_data>"
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
    (cl:setf (cl:slot-value msg 'input_v) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'input_r) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'input_beta) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'input_Tf) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'input_Tr) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'input_delta) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'input_error_v) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'input_error_r) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'input_error_beta) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<state_data>)))
  "Returns string type for a message object of type '<state_data>"
  "pred_msg/state_data")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'state_data)))
  "Returns string type for a message object of type 'state_data"
  "pred_msg/state_data")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<state_data>)))
  "Returns md5sum for a message object of type '<state_data>"
  "daae5795cc22390ee753757c10fc1367")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'state_data)))
  "Returns md5sum for a message object of type 'state_data"
  "daae5795cc22390ee753757c10fc1367")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<state_data>)))
  "Returns full string definition for message of type '<state_data>"
  (cl:format cl:nil "int32 index_data~%float64 input_v~%float64 input_r~%float64 input_beta~%float64 input_Tf~%float64 input_Tr~%float64 input_delta~%float64 input_error_v~%float64 input_error_r~%float64 input_error_beta~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'state_data)))
  "Returns full string definition for message of type 'state_data"
  (cl:format cl:nil "int32 index_data~%float64 input_v~%float64 input_r~%float64 input_beta~%float64 input_Tf~%float64 input_Tr~%float64 input_delta~%float64 input_error_v~%float64 input_error_r~%float64 input_error_beta~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <state_data>))
  (cl:+ 0
     4
     8
     8
     8
     8
     8
     8
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <state_data>))
  "Converts a ROS message object to a list"
  (cl:list 'state_data
    (cl:cons ':index_data (index_data msg))
    (cl:cons ':input_v (input_v msg))
    (cl:cons ':input_r (input_r msg))
    (cl:cons ':input_beta (input_beta msg))
    (cl:cons ':input_Tf (input_Tf msg))
    (cl:cons ':input_Tr (input_Tr msg))
    (cl:cons ':input_delta (input_delta msg))
    (cl:cons ':input_error_v (input_error_v msg))
    (cl:cons ':input_error_r (input_error_r msg))
    (cl:cons ':input_error_beta (input_error_beta msg))
))

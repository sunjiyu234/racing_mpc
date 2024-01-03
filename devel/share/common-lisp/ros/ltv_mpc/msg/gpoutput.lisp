; Auto-generated. Do not edit!


(cl:in-package ltv_mpc-msg)


;//! \htmlinclude gpoutput.msg.html

(cl:defclass <gpoutput> (roslisp-msg-protocol:ros-message)
  ((v_error
    :reader v_error
    :initarg :v_error
    :type cl:float
    :initform 0.0)
   (r_error
    :reader r_error
    :initarg :r_error
    :type cl:float
    :initform 0.0)
   (beta_error
    :reader beta_error
    :initarg :beta_error
    :type cl:float
    :initform 0.0))
)

(cl:defclass gpoutput (<gpoutput>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <gpoutput>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'gpoutput)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ltv_mpc-msg:<gpoutput> is deprecated: use ltv_mpc-msg:gpoutput instead.")))

(cl:ensure-generic-function 'v_error-val :lambda-list '(m))
(cl:defmethod v_error-val ((m <gpoutput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ltv_mpc-msg:v_error-val is deprecated.  Use ltv_mpc-msg:v_error instead.")
  (v_error m))

(cl:ensure-generic-function 'r_error-val :lambda-list '(m))
(cl:defmethod r_error-val ((m <gpoutput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ltv_mpc-msg:r_error-val is deprecated.  Use ltv_mpc-msg:r_error instead.")
  (r_error m))

(cl:ensure-generic-function 'beta_error-val :lambda-list '(m))
(cl:defmethod beta_error-val ((m <gpoutput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ltv_mpc-msg:beta_error-val is deprecated.  Use ltv_mpc-msg:beta_error instead.")
  (beta_error m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <gpoutput>) ostream)
  "Serializes a message object of type '<gpoutput>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'v_error))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'r_error))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'beta_error))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <gpoutput>) istream)
  "Deserializes a message object of type '<gpoutput>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'v_error) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'r_error) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'beta_error) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<gpoutput>)))
  "Returns string type for a message object of type '<gpoutput>"
  "ltv_mpc/gpoutput")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'gpoutput)))
  "Returns string type for a message object of type 'gpoutput"
  "ltv_mpc/gpoutput")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<gpoutput>)))
  "Returns md5sum for a message object of type '<gpoutput>"
  "529d1c55d988fb7bb8a7cea9ad6453f5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'gpoutput)))
  "Returns md5sum for a message object of type 'gpoutput"
  "529d1c55d988fb7bb8a7cea9ad6453f5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<gpoutput>)))
  "Returns full string definition for message of type '<gpoutput>"
  (cl:format cl:nil "float64 v_error~%float64 r_error~%float64 beta_error~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'gpoutput)))
  "Returns full string definition for message of type 'gpoutput"
  (cl:format cl:nil "float64 v_error~%float64 r_error~%float64 beta_error~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <gpoutput>))
  (cl:+ 0
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <gpoutput>))
  "Converts a ROS message object to a list"
  (cl:list 'gpoutput
    (cl:cons ':v_error (v_error msg))
    (cl:cons ':r_error (r_error msg))
    (cl:cons ':beta_error (beta_error msg))
))

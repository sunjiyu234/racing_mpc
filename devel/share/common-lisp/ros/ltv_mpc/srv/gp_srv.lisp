; Auto-generated. Do not edit!


(cl:in-package ltv_mpc-srv)


;//! \htmlinclude gp_srv-request.msg.html

(cl:defclass <gp_srv-request> (roslisp-msg-protocol:ros-message)
  ((gp_input_now
    :reader gp_input_now
    :initarg :gp_input_now
    :type ltv_mpc-msg:gpinput
    :initform (cl:make-instance 'ltv_mpc-msg:gpinput)))
)

(cl:defclass gp_srv-request (<gp_srv-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <gp_srv-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'gp_srv-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ltv_mpc-srv:<gp_srv-request> is deprecated: use ltv_mpc-srv:gp_srv-request instead.")))

(cl:ensure-generic-function 'gp_input_now-val :lambda-list '(m))
(cl:defmethod gp_input_now-val ((m <gp_srv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ltv_mpc-srv:gp_input_now-val is deprecated.  Use ltv_mpc-srv:gp_input_now instead.")
  (gp_input_now m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <gp_srv-request>) ostream)
  "Serializes a message object of type '<gp_srv-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'gp_input_now) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <gp_srv-request>) istream)
  "Deserializes a message object of type '<gp_srv-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'gp_input_now) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<gp_srv-request>)))
  "Returns string type for a service object of type '<gp_srv-request>"
  "ltv_mpc/gp_srvRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'gp_srv-request)))
  "Returns string type for a service object of type 'gp_srv-request"
  "ltv_mpc/gp_srvRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<gp_srv-request>)))
  "Returns md5sum for a message object of type '<gp_srv-request>"
  "44a6e9c0a8d485a5d5e43661bad4e2ba")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'gp_srv-request)))
  "Returns md5sum for a message object of type 'gp_srv-request"
  "44a6e9c0a8d485a5d5e43661bad4e2ba")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<gp_srv-request>)))
  "Returns full string definition for message of type '<gp_srv-request>"
  (cl:format cl:nil "gpinput gp_input_now~%~%================================================================================~%MSG: ltv_mpc/gpinput~%float64 v~%float64 r~%float64 beta~%float64 Tf~%float64 Tr~%float64 Steer~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'gp_srv-request)))
  "Returns full string definition for message of type 'gp_srv-request"
  (cl:format cl:nil "gpinput gp_input_now~%~%================================================================================~%MSG: ltv_mpc/gpinput~%float64 v~%float64 r~%float64 beta~%float64 Tf~%float64 Tr~%float64 Steer~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <gp_srv-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'gp_input_now))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <gp_srv-request>))
  "Converts a ROS message object to a list"
  (cl:list 'gp_srv-request
    (cl:cons ':gp_input_now (gp_input_now msg))
))
;//! \htmlinclude gp_srv-response.msg.html

(cl:defclass <gp_srv-response> (roslisp-msg-protocol:ros-message)
  ((gp_output_now
    :reader gp_output_now
    :initarg :gp_output_now
    :type ltv_mpc-msg:gpoutput
    :initform (cl:make-instance 'ltv_mpc-msg:gpoutput)))
)

(cl:defclass gp_srv-response (<gp_srv-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <gp_srv-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'gp_srv-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ltv_mpc-srv:<gp_srv-response> is deprecated: use ltv_mpc-srv:gp_srv-response instead.")))

(cl:ensure-generic-function 'gp_output_now-val :lambda-list '(m))
(cl:defmethod gp_output_now-val ((m <gp_srv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ltv_mpc-srv:gp_output_now-val is deprecated.  Use ltv_mpc-srv:gp_output_now instead.")
  (gp_output_now m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <gp_srv-response>) ostream)
  "Serializes a message object of type '<gp_srv-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'gp_output_now) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <gp_srv-response>) istream)
  "Deserializes a message object of type '<gp_srv-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'gp_output_now) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<gp_srv-response>)))
  "Returns string type for a service object of type '<gp_srv-response>"
  "ltv_mpc/gp_srvResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'gp_srv-response)))
  "Returns string type for a service object of type 'gp_srv-response"
  "ltv_mpc/gp_srvResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<gp_srv-response>)))
  "Returns md5sum for a message object of type '<gp_srv-response>"
  "44a6e9c0a8d485a5d5e43661bad4e2ba")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'gp_srv-response)))
  "Returns md5sum for a message object of type 'gp_srv-response"
  "44a6e9c0a8d485a5d5e43661bad4e2ba")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<gp_srv-response>)))
  "Returns full string definition for message of type '<gp_srv-response>"
  (cl:format cl:nil "gpoutput gp_output_now~%~%================================================================================~%MSG: ltv_mpc/gpoutput~%float64 v_error~%float64 r_error~%float64 beta_error~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'gp_srv-response)))
  "Returns full string definition for message of type 'gp_srv-response"
  (cl:format cl:nil "gpoutput gp_output_now~%~%================================================================================~%MSG: ltv_mpc/gpoutput~%float64 v_error~%float64 r_error~%float64 beta_error~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <gp_srv-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'gp_output_now))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <gp_srv-response>))
  "Converts a ROS message object to a list"
  (cl:list 'gp_srv-response
    (cl:cons ':gp_output_now (gp_output_now msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'gp_srv)))
  'gp_srv-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'gp_srv)))
  'gp_srv-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'gp_srv)))
  "Returns string type for a service object of type '<gp_srv>"
  "ltv_mpc/gp_srv")
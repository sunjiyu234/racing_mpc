; Auto-generated. Do not edit!


(cl:in-package ltv_mpc-msg)


;//! \htmlinclude gp_data.msg.html

(cl:defclass <gp_data> (roslisp-msg-protocol:ros-message)
  ((gp_input
    :reader gp_input
    :initarg :gp_input
    :type ltv_mpc-msg:gpinput
    :initform (cl:make-instance 'ltv_mpc-msg:gpinput))
   (gp_output
    :reader gp_output
    :initarg :gp_output
    :type ltv_mpc-msg:gpoutput
    :initform (cl:make-instance 'ltv_mpc-msg:gpoutput)))
)

(cl:defclass gp_data (<gp_data>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <gp_data>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'gp_data)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ltv_mpc-msg:<gp_data> is deprecated: use ltv_mpc-msg:gp_data instead.")))

(cl:ensure-generic-function 'gp_input-val :lambda-list '(m))
(cl:defmethod gp_input-val ((m <gp_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ltv_mpc-msg:gp_input-val is deprecated.  Use ltv_mpc-msg:gp_input instead.")
  (gp_input m))

(cl:ensure-generic-function 'gp_output-val :lambda-list '(m))
(cl:defmethod gp_output-val ((m <gp_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ltv_mpc-msg:gp_output-val is deprecated.  Use ltv_mpc-msg:gp_output instead.")
  (gp_output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <gp_data>) ostream)
  "Serializes a message object of type '<gp_data>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'gp_input) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'gp_output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <gp_data>) istream)
  "Deserializes a message object of type '<gp_data>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'gp_input) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'gp_output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<gp_data>)))
  "Returns string type for a message object of type '<gp_data>"
  "ltv_mpc/gp_data")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'gp_data)))
  "Returns string type for a message object of type 'gp_data"
  "ltv_mpc/gp_data")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<gp_data>)))
  "Returns md5sum for a message object of type '<gp_data>"
  "2757ea2368b9f205bb07f6799989923c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'gp_data)))
  "Returns md5sum for a message object of type 'gp_data"
  "2757ea2368b9f205bb07f6799989923c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<gp_data>)))
  "Returns full string definition for message of type '<gp_data>"
  (cl:format cl:nil "gpinput gp_input~%gpoutput gp_output~%================================================================================~%MSG: ltv_mpc/gpinput~%float64 v~%float64 r~%float64 beta~%float64 Tf~%float64 Tr~%float64 Steer~%================================================================================~%MSG: ltv_mpc/gpoutput~%float64 v_error~%float64 r_error~%float64 beta_error~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'gp_data)))
  "Returns full string definition for message of type 'gp_data"
  (cl:format cl:nil "gpinput gp_input~%gpoutput gp_output~%================================================================================~%MSG: ltv_mpc/gpinput~%float64 v~%float64 r~%float64 beta~%float64 Tf~%float64 Tr~%float64 Steer~%================================================================================~%MSG: ltv_mpc/gpoutput~%float64 v_error~%float64 r_error~%float64 beta_error~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <gp_data>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'gp_input))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'gp_output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <gp_data>))
  "Converts a ROS message object to a list"
  (cl:list 'gp_data
    (cl:cons ':gp_input (gp_input msg))
    (cl:cons ':gp_output (gp_output msg))
))

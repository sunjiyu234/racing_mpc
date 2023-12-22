; Auto-generated. Do not edit!


(cl:in-package ltv_mpc-msg)


;//! \htmlinclude sample_lst.msg.html

(cl:defclass <sample_lst> (roslisp-msg-protocol:ros-message)
  ((sample_list
    :reader sample_list
    :initarg :sample_list
    :type (cl:vector ltv_mpc-msg:sample)
   :initform (cl:make-array 0 :element-type 'ltv_mpc-msg:sample :initial-element (cl:make-instance 'ltv_mpc-msg:sample))))
)

(cl:defclass sample_lst (<sample_lst>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <sample_lst>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'sample_lst)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ltv_mpc-msg:<sample_lst> is deprecated: use ltv_mpc-msg:sample_lst instead.")))

(cl:ensure-generic-function 'sample_list-val :lambda-list '(m))
(cl:defmethod sample_list-val ((m <sample_lst>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ltv_mpc-msg:sample_list-val is deprecated.  Use ltv_mpc-msg:sample_list instead.")
  (sample_list m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <sample_lst>) ostream)
  "Serializes a message object of type '<sample_lst>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'sample_list))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'sample_list))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <sample_lst>) istream)
  "Deserializes a message object of type '<sample_lst>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'sample_list) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'sample_list)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'ltv_mpc-msg:sample))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<sample_lst>)))
  "Returns string type for a message object of type '<sample_lst>"
  "ltv_mpc/sample_lst")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'sample_lst)))
  "Returns string type for a message object of type 'sample_lst"
  "ltv_mpc/sample_lst")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<sample_lst>)))
  "Returns md5sum for a message object of type '<sample_lst>"
  "4b7f364dc273676e8884e28e28f4d153")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'sample_lst)))
  "Returns md5sum for a message object of type 'sample_lst"
  "4b7f364dc273676e8884e28e28f4d153")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<sample_lst>)))
  "Returns full string definition for message of type '<sample_lst>"
  (cl:format cl:nil "sample[] sample_list~%================================================================================~%MSG: ltv_mpc/sample~%float64 x~%float64 y~%float64 yaw~%float64 v~%float64 r~%float64 beta~%float64 tf~%float64 tr~%float64 steer~%float64 s~%float64 time~%float64 iter~%float64 cost~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'sample_lst)))
  "Returns full string definition for message of type 'sample_lst"
  (cl:format cl:nil "sample[] sample_list~%================================================================================~%MSG: ltv_mpc/sample~%float64 x~%float64 y~%float64 yaw~%float64 v~%float64 r~%float64 beta~%float64 tf~%float64 tr~%float64 steer~%float64 s~%float64 time~%float64 iter~%float64 cost~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <sample_lst>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'sample_list) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <sample_lst>))
  "Converts a ROS message object to a list"
  (cl:list 'sample_lst
    (cl:cons ':sample_list (sample_list msg))
))

;; Auto-generated. Do not edit!


(when (boundp 'ltv_mpc::gp_data)
  (if (not (find-package "LTV_MPC"))
    (make-package "LTV_MPC"))
  (shadow 'gp_data (find-package "LTV_MPC")))
(unless (find-package "LTV_MPC::GP_DATA")
  (make-package "LTV_MPC::GP_DATA"))

(in-package "ROS")
;;//! \htmlinclude gp_data.msg.html


(defclass ltv_mpc::gp_data
  :super ros::object
  :slots (_gp_input _gp_output ))

(defmethod ltv_mpc::gp_data
  (:init
   (&key
    ((:gp_input __gp_input) (instance ltv_mpc::gpinput :init))
    ((:gp_output __gp_output) (instance ltv_mpc::gpoutput :init))
    )
   (send-super :init)
   (setq _gp_input __gp_input)
   (setq _gp_output __gp_output)
   self)
  (:gp_input
   (&rest __gp_input)
   (if (keywordp (car __gp_input))
       (send* _gp_input __gp_input)
     (progn
       (if __gp_input (setq _gp_input (car __gp_input)))
       _gp_input)))
  (:gp_output
   (&rest __gp_output)
   (if (keywordp (car __gp_output))
       (send* _gp_output __gp_output)
     (progn
       (if __gp_output (setq _gp_output (car __gp_output)))
       _gp_output)))
  (:serialization-length
   ()
   (+
    ;; ltv_mpc/gpinput _gp_input
    (send _gp_input :serialization-length)
    ;; ltv_mpc/gpoutput _gp_output
    (send _gp_output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; ltv_mpc/gpinput _gp_input
       (send _gp_input :serialize s)
     ;; ltv_mpc/gpoutput _gp_output
       (send _gp_output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; ltv_mpc/gpinput _gp_input
     (send _gp_input :deserialize buf ptr-) (incf ptr- (send _gp_input :serialization-length))
   ;; ltv_mpc/gpoutput _gp_output
     (send _gp_output :deserialize buf ptr-) (incf ptr- (send _gp_output :serialization-length))
   ;;
   self)
  )

(setf (get ltv_mpc::gp_data :md5sum-) "2757ea2368b9f205bb07f6799989923c")
(setf (get ltv_mpc::gp_data :datatype-) "ltv_mpc/gp_data")
(setf (get ltv_mpc::gp_data :definition-)
      "gpinput gp_input
gpoutput gp_output
================================================================================
MSG: ltv_mpc/gpinput
float64 v
float64 r
float64 beta
float64 Tf
float64 Tr
float64 Steer
================================================================================
MSG: ltv_mpc/gpoutput
float64 v_error
float64 r_error
float64 beta_error
")



(provide :ltv_mpc/gp_data "2757ea2368b9f205bb07f6799989923c")



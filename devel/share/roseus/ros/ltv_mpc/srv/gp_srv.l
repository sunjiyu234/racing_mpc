;; Auto-generated. Do not edit!


(when (boundp 'ltv_mpc::gp_srv)
  (if (not (find-package "LTV_MPC"))
    (make-package "LTV_MPC"))
  (shadow 'gp_srv (find-package "LTV_MPC")))
(unless (find-package "LTV_MPC::GP_SRV")
  (make-package "LTV_MPC::GP_SRV"))
(unless (find-package "LTV_MPC::GP_SRVREQUEST")
  (make-package "LTV_MPC::GP_SRVREQUEST"))
(unless (find-package "LTV_MPC::GP_SRVRESPONSE")
  (make-package "LTV_MPC::GP_SRVRESPONSE"))

(in-package "ROS")





(defclass ltv_mpc::gp_srvRequest
  :super ros::object
  :slots (_gp_input_now ))

(defmethod ltv_mpc::gp_srvRequest
  (:init
   (&key
    ((:gp_input_now __gp_input_now) (instance ltv_mpc::gpinput :init))
    )
   (send-super :init)
   (setq _gp_input_now __gp_input_now)
   self)
  (:gp_input_now
   (&rest __gp_input_now)
   (if (keywordp (car __gp_input_now))
       (send* _gp_input_now __gp_input_now)
     (progn
       (if __gp_input_now (setq _gp_input_now (car __gp_input_now)))
       _gp_input_now)))
  (:serialization-length
   ()
   (+
    ;; ltv_mpc/gpinput _gp_input_now
    (send _gp_input_now :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; ltv_mpc/gpinput _gp_input_now
       (send _gp_input_now :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; ltv_mpc/gpinput _gp_input_now
     (send _gp_input_now :deserialize buf ptr-) (incf ptr- (send _gp_input_now :serialization-length))
   ;;
   self)
  )

(defclass ltv_mpc::gp_srvResponse
  :super ros::object
  :slots (_gp_output_now ))

(defmethod ltv_mpc::gp_srvResponse
  (:init
   (&key
    ((:gp_output_now __gp_output_now) (instance ltv_mpc::gpoutput :init))
    )
   (send-super :init)
   (setq _gp_output_now __gp_output_now)
   self)
  (:gp_output_now
   (&rest __gp_output_now)
   (if (keywordp (car __gp_output_now))
       (send* _gp_output_now __gp_output_now)
     (progn
       (if __gp_output_now (setq _gp_output_now (car __gp_output_now)))
       _gp_output_now)))
  (:serialization-length
   ()
   (+
    ;; ltv_mpc/gpoutput _gp_output_now
    (send _gp_output_now :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; ltv_mpc/gpoutput _gp_output_now
       (send _gp_output_now :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; ltv_mpc/gpoutput _gp_output_now
     (send _gp_output_now :deserialize buf ptr-) (incf ptr- (send _gp_output_now :serialization-length))
   ;;
   self)
  )

(defclass ltv_mpc::gp_srv
  :super ros::object
  :slots ())

(setf (get ltv_mpc::gp_srv :md5sum-) "44a6e9c0a8d485a5d5e43661bad4e2ba")
(setf (get ltv_mpc::gp_srv :datatype-) "ltv_mpc/gp_srv")
(setf (get ltv_mpc::gp_srv :request) ltv_mpc::gp_srvRequest)
(setf (get ltv_mpc::gp_srv :response) ltv_mpc::gp_srvResponse)

(defmethod ltv_mpc::gp_srvRequest
  (:response () (instance ltv_mpc::gp_srvResponse :init)))

(setf (get ltv_mpc::gp_srvRequest :md5sum-) "44a6e9c0a8d485a5d5e43661bad4e2ba")
(setf (get ltv_mpc::gp_srvRequest :datatype-) "ltv_mpc/gp_srvRequest")
(setf (get ltv_mpc::gp_srvRequest :definition-)
      "gpinput gp_input_now

================================================================================
MSG: ltv_mpc/gpinput
float64 v
float64 r
float64 beta
float64 Tf
float64 Tr
---
gpoutput gp_output_now

================================================================================
MSG: ltv_mpc/gpoutput
float64 v_error
float64 r_error
")

(setf (get ltv_mpc::gp_srvResponse :md5sum-) "44a6e9c0a8d485a5d5e43661bad4e2ba")
(setf (get ltv_mpc::gp_srvResponse :datatype-) "ltv_mpc/gp_srvResponse")
(setf (get ltv_mpc::gp_srvResponse :definition-)
      "gpinput gp_input_now

================================================================================
MSG: ltv_mpc/gpinput
float64 v
float64 r
float64 beta
float64 Tf
float64 Tr
---
gpoutput gp_output_now

================================================================================
MSG: ltv_mpc/gpoutput
float64 v_error
float64 r_error
")



(provide :ltv_mpc/gp_srv "44a6e9c0a8d485a5d5e43661bad4e2ba")



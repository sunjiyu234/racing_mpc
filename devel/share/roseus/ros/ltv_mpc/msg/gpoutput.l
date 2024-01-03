;; Auto-generated. Do not edit!


(when (boundp 'ltv_mpc::gpoutput)
  (if (not (find-package "LTV_MPC"))
    (make-package "LTV_MPC"))
  (shadow 'gpoutput (find-package "LTV_MPC")))
(unless (find-package "LTV_MPC::GPOUTPUT")
  (make-package "LTV_MPC::GPOUTPUT"))

(in-package "ROS")
;;//! \htmlinclude gpoutput.msg.html


(defclass ltv_mpc::gpoutput
  :super ros::object
  :slots (_v_error _r_error _beta_error ))

(defmethod ltv_mpc::gpoutput
  (:init
   (&key
    ((:v_error __v_error) 0.0)
    ((:r_error __r_error) 0.0)
    ((:beta_error __beta_error) 0.0)
    )
   (send-super :init)
   (setq _v_error (float __v_error))
   (setq _r_error (float __r_error))
   (setq _beta_error (float __beta_error))
   self)
  (:v_error
   (&optional __v_error)
   (if __v_error (setq _v_error __v_error)) _v_error)
  (:r_error
   (&optional __r_error)
   (if __r_error (setq _r_error __r_error)) _r_error)
  (:beta_error
   (&optional __beta_error)
   (if __beta_error (setq _beta_error __beta_error)) _beta_error)
  (:serialization-length
   ()
   (+
    ;; float64 _v_error
    8
    ;; float64 _r_error
    8
    ;; float64 _beta_error
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _v_error
       (sys::poke _v_error (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _r_error
       (sys::poke _r_error (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _beta_error
       (sys::poke _beta_error (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _v_error
     (setq _v_error (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _r_error
     (setq _r_error (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _beta_error
     (setq _beta_error (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get ltv_mpc::gpoutput :md5sum-) "529d1c55d988fb7bb8a7cea9ad6453f5")
(setf (get ltv_mpc::gpoutput :datatype-) "ltv_mpc/gpoutput")
(setf (get ltv_mpc::gpoutput :definition-)
      "float64 v_error
float64 r_error
float64 beta_error
")



(provide :ltv_mpc/gpoutput "529d1c55d988fb7bb8a7cea9ad6453f5")



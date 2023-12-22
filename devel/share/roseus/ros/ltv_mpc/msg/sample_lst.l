;; Auto-generated. Do not edit!


(when (boundp 'ltv_mpc::sample_lst)
  (if (not (find-package "LTV_MPC"))
    (make-package "LTV_MPC"))
  (shadow 'sample_lst (find-package "LTV_MPC")))
(unless (find-package "LTV_MPC::SAMPLE_LST")
  (make-package "LTV_MPC::SAMPLE_LST"))

(in-package "ROS")
;;//! \htmlinclude sample_lst.msg.html


(defclass ltv_mpc::sample_lst
  :super ros::object
  :slots (_sample_list ))

(defmethod ltv_mpc::sample_lst
  (:init
   (&key
    ((:sample_list __sample_list) (let (r) (dotimes (i 0) (push (instance ltv_mpc::sample :init) r)) r))
    )
   (send-super :init)
   (setq _sample_list __sample_list)
   self)
  (:sample_list
   (&rest __sample_list)
   (if (keywordp (car __sample_list))
       (send* _sample_list __sample_list)
     (progn
       (if __sample_list (setq _sample_list (car __sample_list)))
       _sample_list)))
  (:serialization-length
   ()
   (+
    ;; ltv_mpc/sample[] _sample_list
    (apply #'+ (send-all _sample_list :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; ltv_mpc/sample[] _sample_list
     (write-long (length _sample_list) s)
     (dolist (elem _sample_list)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; ltv_mpc/sample[] _sample_list
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _sample_list (let (r) (dotimes (i n) (push (instance ltv_mpc::sample :init) r)) r))
     (dolist (elem- _sample_list)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get ltv_mpc::sample_lst :md5sum-) "4b7f364dc273676e8884e28e28f4d153")
(setf (get ltv_mpc::sample_lst :datatype-) "ltv_mpc/sample_lst")
(setf (get ltv_mpc::sample_lst :definition-)
      "sample[] sample_list
================================================================================
MSG: ltv_mpc/sample
float64 x
float64 y
float64 yaw
float64 v
float64 r
float64 beta
float64 tf
float64 tr
float64 steer
float64 s
float64 time
float64 iter
float64 cost
")



(provide :ltv_mpc/sample_lst "4b7f364dc273676e8884e28e28f4d153")



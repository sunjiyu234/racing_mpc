;; Auto-generated. Do not edit!


(when (boundp 'ltv_mpc::gpinput)
  (if (not (find-package "LTV_MPC"))
    (make-package "LTV_MPC"))
  (shadow 'gpinput (find-package "LTV_MPC")))
(unless (find-package "LTV_MPC::GPINPUT")
  (make-package "LTV_MPC::GPINPUT"))

(in-package "ROS")
;;//! \htmlinclude gpinput.msg.html


(defclass ltv_mpc::gpinput
  :super ros::object
  :slots (_v _r _beta _Tf _Tr _Steer ))

(defmethod ltv_mpc::gpinput
  (:init
   (&key
    ((:v __v) 0.0)
    ((:r __r) 0.0)
    ((:beta __beta) 0.0)
    ((:Tf __Tf) 0.0)
    ((:Tr __Tr) 0.0)
    ((:Steer __Steer) 0.0)
    )
   (send-super :init)
   (setq _v (float __v))
   (setq _r (float __r))
   (setq _beta (float __beta))
   (setq _Tf (float __Tf))
   (setq _Tr (float __Tr))
   (setq _Steer (float __Steer))
   self)
  (:v
   (&optional __v)
   (if __v (setq _v __v)) _v)
  (:r
   (&optional __r)
   (if __r (setq _r __r)) _r)
  (:beta
   (&optional __beta)
   (if __beta (setq _beta __beta)) _beta)
  (:Tf
   (&optional __Tf)
   (if __Tf (setq _Tf __Tf)) _Tf)
  (:Tr
   (&optional __Tr)
   (if __Tr (setq _Tr __Tr)) _Tr)
  (:Steer
   (&optional __Steer)
   (if __Steer (setq _Steer __Steer)) _Steer)
  (:serialization-length
   ()
   (+
    ;; float64 _v
    8
    ;; float64 _r
    8
    ;; float64 _beta
    8
    ;; float64 _Tf
    8
    ;; float64 _Tr
    8
    ;; float64 _Steer
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _v
       (sys::poke _v (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _r
       (sys::poke _r (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _beta
       (sys::poke _beta (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _Tf
       (sys::poke _Tf (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _Tr
       (sys::poke _Tr (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _Steer
       (sys::poke _Steer (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _v
     (setq _v (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _r
     (setq _r (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _beta
     (setq _beta (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _Tf
     (setq _Tf (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _Tr
     (setq _Tr (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _Steer
     (setq _Steer (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get ltv_mpc::gpinput :md5sum-) "dfd53f116b4d87a2c94142f47b445347")
(setf (get ltv_mpc::gpinput :datatype-) "ltv_mpc/gpinput")
(setf (get ltv_mpc::gpinput :definition-)
      "float64 v
float64 r
float64 beta
float64 Tf
float64 Tr
float64 Steer
")



(provide :ltv_mpc/gpinput "dfd53f116b4d87a2c94142f47b445347")



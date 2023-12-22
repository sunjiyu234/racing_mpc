;; Auto-generated. Do not edit!


(when (boundp 'ltv_mpc::sample)
  (if (not (find-package "LTV_MPC"))
    (make-package "LTV_MPC"))
  (shadow 'sample (find-package "LTV_MPC")))
(unless (find-package "LTV_MPC::SAMPLE")
  (make-package "LTV_MPC::SAMPLE"))

(in-package "ROS")
;;//! \htmlinclude sample.msg.html


(defclass ltv_mpc::sample
  :super ros::object
  :slots (_x _y _yaw _v _r _beta _tf _tr _steer _s _time _iter _cost ))

(defmethod ltv_mpc::sample
  (:init
   (&key
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:yaw __yaw) 0.0)
    ((:v __v) 0.0)
    ((:r __r) 0.0)
    ((:beta __beta) 0.0)
    ((:tf __tf) 0.0)
    ((:tr __tr) 0.0)
    ((:steer __steer) 0.0)
    ((:s __s) 0.0)
    ((:time __time) 0.0)
    ((:iter __iter) 0.0)
    ((:cost __cost) 0.0)
    )
   (send-super :init)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _yaw (float __yaw))
   (setq _v (float __v))
   (setq _r (float __r))
   (setq _beta (float __beta))
   (setq _tf (float __tf))
   (setq _tr (float __tr))
   (setq _steer (float __steer))
   (setq _s (float __s))
   (setq _time (float __time))
   (setq _iter (float __iter))
   (setq _cost (float __cost))
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:yaw
   (&optional __yaw)
   (if __yaw (setq _yaw __yaw)) _yaw)
  (:v
   (&optional __v)
   (if __v (setq _v __v)) _v)
  (:r
   (&optional __r)
   (if __r (setq _r __r)) _r)
  (:beta
   (&optional __beta)
   (if __beta (setq _beta __beta)) _beta)
  (:tf
   (&optional __tf)
   (if __tf (setq _tf __tf)) _tf)
  (:tr
   (&optional __tr)
   (if __tr (setq _tr __tr)) _tr)
  (:steer
   (&optional __steer)
   (if __steer (setq _steer __steer)) _steer)
  (:s
   (&optional __s)
   (if __s (setq _s __s)) _s)
  (:time
   (&optional __time)
   (if __time (setq _time __time)) _time)
  (:iter
   (&optional __iter)
   (if __iter (setq _iter __iter)) _iter)
  (:cost
   (&optional __cost)
   (if __cost (setq _cost __cost)) _cost)
  (:serialization-length
   ()
   (+
    ;; float64 _x
    8
    ;; float64 _y
    8
    ;; float64 _yaw
    8
    ;; float64 _v
    8
    ;; float64 _r
    8
    ;; float64 _beta
    8
    ;; float64 _tf
    8
    ;; float64 _tr
    8
    ;; float64 _steer
    8
    ;; float64 _s
    8
    ;; float64 _time
    8
    ;; float64 _iter
    8
    ;; float64 _cost
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _x
       (sys::poke _x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y
       (sys::poke _y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _yaw
       (sys::poke _yaw (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _v
       (sys::poke _v (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _r
       (sys::poke _r (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _beta
       (sys::poke _beta (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _tf
       (sys::poke _tf (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _tr
       (sys::poke _tr (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _steer
       (sys::poke _steer (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _s
       (sys::poke _s (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _time
       (sys::poke _time (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _iter
       (sys::poke _iter (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _cost
       (sys::poke _cost (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _x
     (setq _x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y
     (setq _y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _yaw
     (setq _yaw (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _v
     (setq _v (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _r
     (setq _r (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _beta
     (setq _beta (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _tf
     (setq _tf (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _tr
     (setq _tr (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _steer
     (setq _steer (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _s
     (setq _s (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _time
     (setq _time (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _iter
     (setq _iter (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _cost
     (setq _cost (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get ltv_mpc::sample :md5sum-) "70403b52d11f6e3bff793167877b89ed")
(setf (get ltv_mpc::sample :datatype-) "ltv_mpc/sample")
(setf (get ltv_mpc::sample :definition-)
      "float64 x
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



(provide :ltv_mpc/sample "70403b52d11f6e3bff793167877b89ed")



;; Auto-generated. Do not edit!


(when (boundp 'pred_msg::state_data)
  (if (not (find-package "PRED_MSG"))
    (make-package "PRED_MSG"))
  (shadow 'state_data (find-package "PRED_MSG")))
(unless (find-package "PRED_MSG::STATE_DATA")
  (make-package "PRED_MSG::STATE_DATA"))

(in-package "ROS")
;;//! \htmlinclude state_data.msg.html


(defclass pred_msg::state_data
  :super ros::object
  :slots (_index_data _input_v _input_r _input_beta _input_Tf _input_Tr _input_delta _input_error_v _input_error_r _input_error_beta ))

(defmethod pred_msg::state_data
  (:init
   (&key
    ((:index_data __index_data) 0)
    ((:input_v __input_v) 0.0)
    ((:input_r __input_r) 0.0)
    ((:input_beta __input_beta) 0.0)
    ((:input_Tf __input_Tf) 0.0)
    ((:input_Tr __input_Tr) 0.0)
    ((:input_delta __input_delta) 0.0)
    ((:input_error_v __input_error_v) 0.0)
    ((:input_error_r __input_error_r) 0.0)
    ((:input_error_beta __input_error_beta) 0.0)
    )
   (send-super :init)
   (setq _index_data (round __index_data))
   (setq _input_v (float __input_v))
   (setq _input_r (float __input_r))
   (setq _input_beta (float __input_beta))
   (setq _input_Tf (float __input_Tf))
   (setq _input_Tr (float __input_Tr))
   (setq _input_delta (float __input_delta))
   (setq _input_error_v (float __input_error_v))
   (setq _input_error_r (float __input_error_r))
   (setq _input_error_beta (float __input_error_beta))
   self)
  (:index_data
   (&optional __index_data)
   (if __index_data (setq _index_data __index_data)) _index_data)
  (:input_v
   (&optional __input_v)
   (if __input_v (setq _input_v __input_v)) _input_v)
  (:input_r
   (&optional __input_r)
   (if __input_r (setq _input_r __input_r)) _input_r)
  (:input_beta
   (&optional __input_beta)
   (if __input_beta (setq _input_beta __input_beta)) _input_beta)
  (:input_Tf
   (&optional __input_Tf)
   (if __input_Tf (setq _input_Tf __input_Tf)) _input_Tf)
  (:input_Tr
   (&optional __input_Tr)
   (if __input_Tr (setq _input_Tr __input_Tr)) _input_Tr)
  (:input_delta
   (&optional __input_delta)
   (if __input_delta (setq _input_delta __input_delta)) _input_delta)
  (:input_error_v
   (&optional __input_error_v)
   (if __input_error_v (setq _input_error_v __input_error_v)) _input_error_v)
  (:input_error_r
   (&optional __input_error_r)
   (if __input_error_r (setq _input_error_r __input_error_r)) _input_error_r)
  (:input_error_beta
   (&optional __input_error_beta)
   (if __input_error_beta (setq _input_error_beta __input_error_beta)) _input_error_beta)
  (:serialization-length
   ()
   (+
    ;; int32 _index_data
    4
    ;; float64 _input_v
    8
    ;; float64 _input_r
    8
    ;; float64 _input_beta
    8
    ;; float64 _input_Tf
    8
    ;; float64 _input_Tr
    8
    ;; float64 _input_delta
    8
    ;; float64 _input_error_v
    8
    ;; float64 _input_error_r
    8
    ;; float64 _input_error_beta
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _index_data
       (write-long _index_data s)
     ;; float64 _input_v
       (sys::poke _input_v (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _input_r
       (sys::poke _input_r (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _input_beta
       (sys::poke _input_beta (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _input_Tf
       (sys::poke _input_Tf (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _input_Tr
       (sys::poke _input_Tr (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _input_delta
       (sys::poke _input_delta (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _input_error_v
       (sys::poke _input_error_v (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _input_error_r
       (sys::poke _input_error_r (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _input_error_beta
       (sys::poke _input_error_beta (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _index_data
     (setq _index_data (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float64 _input_v
     (setq _input_v (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _input_r
     (setq _input_r (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _input_beta
     (setq _input_beta (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _input_Tf
     (setq _input_Tf (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _input_Tr
     (setq _input_Tr (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _input_delta
     (setq _input_delta (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _input_error_v
     (setq _input_error_v (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _input_error_r
     (setq _input_error_r (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _input_error_beta
     (setq _input_error_beta (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get pred_msg::state_data :md5sum-) "daae5795cc22390ee753757c10fc1367")
(setf (get pred_msg::state_data :datatype-) "pred_msg/state_data")
(setf (get pred_msg::state_data :definition-)
      "int32 index_data
float64 input_v
float64 input_r
float64 input_beta
float64 input_Tf
float64 input_Tr
float64 input_delta
float64 input_error_v
float64 input_error_r
float64 input_error_beta

")



(provide :pred_msg/state_data "daae5795cc22390ee753757c10fc1367")



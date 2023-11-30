;; Auto-generated. Do not edit!


(when (boundp 'pred_msg::correct_data)
  (if (not (find-package "PRED_MSG"))
    (make-package "PRED_MSG"))
  (shadow 'correct_data (find-package "PRED_MSG")))
(unless (find-package "PRED_MSG::CORRECT_DATA")
  (make-package "PRED_MSG::CORRECT_DATA"))

(in-package "ROS")
;;//! \htmlinclude correct_data.msg.html


(defclass pred_msg::correct_data
  :super ros::object
  :slots (_index_data _correct_v _correct_r _correct_beta ))

(defmethod pred_msg::correct_data
  (:init
   (&key
    ((:index_data __index_data) 0)
    ((:correct_v __correct_v) 0.0)
    ((:correct_r __correct_r) 0.0)
    ((:correct_beta __correct_beta) 0.0)
    )
   (send-super :init)
   (setq _index_data (round __index_data))
   (setq _correct_v (float __correct_v))
   (setq _correct_r (float __correct_r))
   (setq _correct_beta (float __correct_beta))
   self)
  (:index_data
   (&optional __index_data)
   (if __index_data (setq _index_data __index_data)) _index_data)
  (:correct_v
   (&optional __correct_v)
   (if __correct_v (setq _correct_v __correct_v)) _correct_v)
  (:correct_r
   (&optional __correct_r)
   (if __correct_r (setq _correct_r __correct_r)) _correct_r)
  (:correct_beta
   (&optional __correct_beta)
   (if __correct_beta (setq _correct_beta __correct_beta)) _correct_beta)
  (:serialization-length
   ()
   (+
    ;; int32 _index_data
    4
    ;; float64 _correct_v
    8
    ;; float64 _correct_r
    8
    ;; float64 _correct_beta
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _index_data
       (write-long _index_data s)
     ;; float64 _correct_v
       (sys::poke _correct_v (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _correct_r
       (sys::poke _correct_r (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _correct_beta
       (sys::poke _correct_beta (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _index_data
     (setq _index_data (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float64 _correct_v
     (setq _correct_v (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _correct_r
     (setq _correct_r (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _correct_beta
     (setq _correct_beta (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get pred_msg::correct_data :md5sum-) "41f364437c141998133be277ae3d31c7")
(setf (get pred_msg::correct_data :datatype-) "pred_msg/correct_data")
(setf (get pred_msg::correct_data :definition-)
      "int32 index_data
float64 correct_v
float64 correct_r
float64 correct_beta

")



(provide :pred_msg/correct_data "41f364437c141998133be277ae3d31c7")



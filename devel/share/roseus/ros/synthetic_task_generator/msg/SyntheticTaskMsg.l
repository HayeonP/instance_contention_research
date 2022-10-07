;; Auto-generated. Do not edit!


(when (boundp 'synthetic_task_generator::SyntheticTaskMsg)
  (if (not (find-package "SYNTHETIC_TASK_GENERATOR"))
    (make-package "SYNTHETIC_TASK_GENERATOR"))
  (shadow 'SyntheticTaskMsg (find-package "SYNTHETIC_TASK_GENERATOR")))
(unless (find-package "SYNTHETIC_TASK_GENERATOR::SYNTHETICTASKMSG")
  (make-package "SYNTHETIC_TASK_GENERATOR::SYNTHETICTASKMSG"))

(in-package "ROS")
;;//! \htmlinclude SyntheticTaskMsg.msg.html


(defclass synthetic_task_generator::SyntheticTaskMsg
  :super ros::object
  :slots (_instance _value ))

(defmethod synthetic_task_generator::SyntheticTaskMsg
  (:init
   (&key
    ((:instance __instance) 0)
    ((:value __value) 0.0)
    )
   (send-super :init)
   (setq _instance (round __instance))
   (setq _value (float __value))
   self)
  (:instance
   (&optional __instance)
   (if __instance (setq _instance __instance)) _instance)
  (:value
   (&optional __value)
   (if __value (setq _value __value)) _value)
  (:serialization-length
   ()
   (+
    ;; int64 _instance
    8
    ;; float64 _value
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64 _instance
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _instance (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _instance) (= (length (_instance . bv)) 2)) ;; bignum
              (write-long (ash (elt (_instance . bv) 0) 0) s)
              (write-long (ash (elt (_instance . bv) 1) -1) s))
             ((and (class _instance) (= (length (_instance . bv)) 1)) ;; big1
              (write-long (elt (_instance . bv) 0) s)
              (write-long (if (>= _instance 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _instance s)(write-long (if (>= _instance 0) 0 #xffffffff) s)))
     ;; float64 _value
       (sys::poke _value (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64 _instance
#+(or :alpha :irix6 :x86_64)
      (setf _instance (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _instance (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; float64 _value
     (setq _value (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get synthetic_task_generator::SyntheticTaskMsg :md5sum-) "cbfe4c158f332960f1182cd0ed832cbc")
(setf (get synthetic_task_generator::SyntheticTaskMsg :datatype-) "synthetic_task_generator/SyntheticTaskMsg")
(setf (get synthetic_task_generator::SyntheticTaskMsg :definition-)
      "int64 instance
float64 value
")



(provide :synthetic_task_generator/SyntheticTaskMsg "cbfe4c158f332960f1182cd0ed832cbc")



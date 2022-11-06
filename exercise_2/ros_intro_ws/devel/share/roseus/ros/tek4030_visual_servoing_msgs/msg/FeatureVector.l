;; Auto-generated. Do not edit!


(when (boundp 'tek4030_visual_servoing_msgs::FeatureVector)
  (if (not (find-package "TEK4030_VISUAL_SERVOING_MSGS"))
    (make-package "TEK4030_VISUAL_SERVOING_MSGS"))
  (shadow 'FeatureVector (find-package "TEK4030_VISUAL_SERVOING_MSGS")))
(unless (find-package "TEK4030_VISUAL_SERVOING_MSGS::FEATUREVECTOR")
  (make-package "TEK4030_VISUAL_SERVOING_MSGS::FEATUREVECTOR"))

(in-package "ROS")
;;//! \htmlinclude FeatureVector.msg.html


(defclass tek4030_visual_servoing_msgs::FeatureVector
  :super ros::object
  :slots (_s ))

(defmethod tek4030_visual_servoing_msgs::FeatureVector
  (:init
   (&key
    ((:s __s) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _s __s)
   self)
  (:s
   (&optional __s)
   (if __s (setq _s __s)) _s)
  (:serialization-length
   ()
   (+
    ;; float64[] _s
    (* 8    (length _s)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[] _s
     (write-long (length _s) s)
     (dotimes (i (length _s))
       (sys::poke (elt _s i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[] _s
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _s (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _s i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get tek4030_visual_servoing_msgs::FeatureVector :md5sum-) "d78041f69764b98461f668e20573e9de")
(setf (get tek4030_visual_servoing_msgs::FeatureVector :datatype-) "tek4030_visual_servoing_msgs/FeatureVector")
(setf (get tek4030_visual_servoing_msgs::FeatureVector :definition-)
      "float64[] s

")



(provide :tek4030_visual_servoing_msgs/FeatureVector "d78041f69764b98461f668e20573e9de")



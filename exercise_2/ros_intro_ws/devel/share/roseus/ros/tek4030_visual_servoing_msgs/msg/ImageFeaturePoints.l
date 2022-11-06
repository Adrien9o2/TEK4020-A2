;; Auto-generated. Do not edit!


(when (boundp 'tek4030_visual_servoing_msgs::ImageFeaturePoints)
  (if (not (find-package "TEK4030_VISUAL_SERVOING_MSGS"))
    (make-package "TEK4030_VISUAL_SERVOING_MSGS"))
  (shadow 'ImageFeaturePoints (find-package "TEK4030_VISUAL_SERVOING_MSGS")))
(unless (find-package "TEK4030_VISUAL_SERVOING_MSGS::IMAGEFEATUREPOINTS")
  (make-package "TEK4030_VISUAL_SERVOING_MSGS::IMAGEFEATUREPOINTS"))

(in-package "ROS")
;;//! \htmlinclude ImageFeaturePoints.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass tek4030_visual_servoing_msgs::ImageFeaturePoints
  :super ros::object
  :slots (_p ))

(defmethod tek4030_visual_servoing_msgs::ImageFeaturePoints
  (:init
   (&key
    ((:p __p) (let (r) (dotimes (i 0) (push (instance geometry_msgs::Point :init) r)) r))
    )
   (send-super :init)
   (setq _p __p)
   self)
  (:p
   (&rest __p)
   (if (keywordp (car __p))
       (send* _p __p)
     (progn
       (if __p (setq _p (car __p)))
       _p)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Point[] _p
    (apply #'+ (send-all _p :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Point[] _p
     (write-long (length _p) s)
     (dolist (elem _p)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Point[] _p
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _p (let (r) (dotimes (i n) (push (instance geometry_msgs::Point :init) r)) r))
     (dolist (elem- _p)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get tek4030_visual_servoing_msgs::ImageFeaturePoints :md5sum-) "74ca570665a40606dd930ab3deca3e02")
(setf (get tek4030_visual_servoing_msgs::ImageFeaturePoints :datatype-) "tek4030_visual_servoing_msgs/ImageFeaturePoints")
(setf (get tek4030_visual_servoing_msgs::ImageFeaturePoints :definition-)
      "geometry_msgs/Point[] p

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :tek4030_visual_servoing_msgs/ImageFeaturePoints "74ca570665a40606dd930ab3deca3e02")



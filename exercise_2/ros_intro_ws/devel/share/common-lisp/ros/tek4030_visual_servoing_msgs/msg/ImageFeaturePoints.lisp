; Auto-generated. Do not edit!


(cl:in-package tek4030_visual_servoing_msgs-msg)


;//! \htmlinclude ImageFeaturePoints.msg.html

(cl:defclass <ImageFeaturePoints> (roslisp-msg-protocol:ros-message)
  ((p
    :reader p
    :initarg :p
    :type (cl:vector geometry_msgs-msg:Point)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point :initial-element (cl:make-instance 'geometry_msgs-msg:Point))))
)

(cl:defclass ImageFeaturePoints (<ImageFeaturePoints>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ImageFeaturePoints>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ImageFeaturePoints)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tek4030_visual_servoing_msgs-msg:<ImageFeaturePoints> is deprecated: use tek4030_visual_servoing_msgs-msg:ImageFeaturePoints instead.")))

(cl:ensure-generic-function 'p-val :lambda-list '(m))
(cl:defmethod p-val ((m <ImageFeaturePoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tek4030_visual_servoing_msgs-msg:p-val is deprecated.  Use tek4030_visual_servoing_msgs-msg:p instead.")
  (p m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ImageFeaturePoints>) ostream)
  "Serializes a message object of type '<ImageFeaturePoints>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'p))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'p))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ImageFeaturePoints>) istream)
  "Deserializes a message object of type '<ImageFeaturePoints>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'p) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'p)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ImageFeaturePoints>)))
  "Returns string type for a message object of type '<ImageFeaturePoints>"
  "tek4030_visual_servoing_msgs/ImageFeaturePoints")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ImageFeaturePoints)))
  "Returns string type for a message object of type 'ImageFeaturePoints"
  "tek4030_visual_servoing_msgs/ImageFeaturePoints")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ImageFeaturePoints>)))
  "Returns md5sum for a message object of type '<ImageFeaturePoints>"
  "74ca570665a40606dd930ab3deca3e02")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ImageFeaturePoints)))
  "Returns md5sum for a message object of type 'ImageFeaturePoints"
  "74ca570665a40606dd930ab3deca3e02")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ImageFeaturePoints>)))
  "Returns full string definition for message of type '<ImageFeaturePoints>"
  (cl:format cl:nil "geometry_msgs/Point[] p~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ImageFeaturePoints)))
  "Returns full string definition for message of type 'ImageFeaturePoints"
  (cl:format cl:nil "geometry_msgs/Point[] p~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ImageFeaturePoints>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'p) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ImageFeaturePoints>))
  "Converts a ROS message object to a list"
  (cl:list 'ImageFeaturePoints
    (cl:cons ':p (p msg))
))

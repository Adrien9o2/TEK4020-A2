; Auto-generated. Do not edit!


(cl:in-package tek4030_visual_servoing_msgs-msg)


;//! \htmlinclude FeatureVector.msg.html

(cl:defclass <FeatureVector> (roslisp-msg-protocol:ros-message)
  ((s
    :reader s
    :initarg :s
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass FeatureVector (<FeatureVector>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FeatureVector>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FeatureVector)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tek4030_visual_servoing_msgs-msg:<FeatureVector> is deprecated: use tek4030_visual_servoing_msgs-msg:FeatureVector instead.")))

(cl:ensure-generic-function 's-val :lambda-list '(m))
(cl:defmethod s-val ((m <FeatureVector>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tek4030_visual_servoing_msgs-msg:s-val is deprecated.  Use tek4030_visual_servoing_msgs-msg:s instead.")
  (s m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FeatureVector>) ostream)
  "Serializes a message object of type '<FeatureVector>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 's))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 's))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FeatureVector>) istream)
  "Deserializes a message object of type '<FeatureVector>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 's) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 's)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FeatureVector>)))
  "Returns string type for a message object of type '<FeatureVector>"
  "tek4030_visual_servoing_msgs/FeatureVector")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FeatureVector)))
  "Returns string type for a message object of type 'FeatureVector"
  "tek4030_visual_servoing_msgs/FeatureVector")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FeatureVector>)))
  "Returns md5sum for a message object of type '<FeatureVector>"
  "d78041f69764b98461f668e20573e9de")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FeatureVector)))
  "Returns md5sum for a message object of type 'FeatureVector"
  "d78041f69764b98461f668e20573e9de")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FeatureVector>)))
  "Returns full string definition for message of type '<FeatureVector>"
  (cl:format cl:nil "float64[] s~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FeatureVector)))
  "Returns full string definition for message of type 'FeatureVector"
  (cl:format cl:nil "float64[] s~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FeatureVector>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 's) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FeatureVector>))
  "Converts a ROS message object to a list"
  (cl:list 'FeatureVector
    (cl:cons ':s (s msg))
))

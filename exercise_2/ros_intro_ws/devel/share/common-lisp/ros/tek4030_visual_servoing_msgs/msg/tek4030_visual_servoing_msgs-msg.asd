
(cl:in-package :asdf)

(defsystem "tek4030_visual_servoing_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "FeatureVector" :depends-on ("_package_FeatureVector"))
    (:file "_package_FeatureVector" :depends-on ("_package"))
    (:file "ImageFeaturePoints" :depends-on ("_package_ImageFeaturePoints"))
    (:file "_package_ImageFeaturePoints" :depends-on ("_package"))
  ))
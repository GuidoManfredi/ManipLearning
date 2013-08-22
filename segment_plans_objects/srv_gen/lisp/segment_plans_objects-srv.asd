
(cl:in-package :asdf)

(defsystem "segment_plans_objects-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :sensor_msgs-msg
               :tabletop_object_detector-msg
)
  :components ((:file "_package")
    (:file "PlantopSegmentation" :depends-on ("_package_PlantopSegmentation"))
    (:file "_package_PlantopSegmentation" :depends-on ("_package"))
  ))
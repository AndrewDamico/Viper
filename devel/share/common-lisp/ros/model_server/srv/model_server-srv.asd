
(cl:in-package :asdf)

(defsystem "model_server-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :model_server-msg
               :sensor_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "ImageRequest" :depends-on ("_package_ImageRequest"))
    (:file "_package_ImageRequest" :depends-on ("_package"))
    (:file "ModelRequest" :depends-on ("_package_ModelRequest"))
    (:file "_package_ModelRequest" :depends-on ("_package"))
  ))
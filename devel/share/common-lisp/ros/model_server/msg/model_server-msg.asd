
(cl:in-package :asdf)

(defsystem "model_server-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "InferenceResults" :depends-on ("_package_InferenceResults"))
    (:file "_package_InferenceResults" :depends-on ("_package"))
  ))
;; Auto-generated. Do not edit!


(when (boundp 'model_server::InferenceResults)
  (if (not (find-package "MODEL_SERVER"))
    (make-package "MODEL_SERVER"))
  (shadow 'InferenceResults (find-package "MODEL_SERVER")))
(unless (find-package "MODEL_SERVER::INFERENCERESULTS")
  (make-package "MODEL_SERVER::INFERENCERESULTS"))

(in-package "ROS")
;;//! \htmlinclude InferenceResults.msg.html


(defclass model_server::InferenceResults
  :super ros::object
  :slots (_structure _inferences ))

(defmethod model_server::InferenceResults
  (:init
   (&key
    ((:structure __structure) (make-array 0 :initial-element 0 :element-type :integer))
    ((:inferences __inferences) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _structure __structure)
   (setq _inferences __inferences)
   self)
  (:structure
   (&optional __structure)
   (if __structure (setq _structure __structure)) _structure)
  (:inferences
   (&optional __inferences)
   (if __inferences (setq _inferences __inferences)) _inferences)
  (:serialization-length
   ()
   (+
    ;; int32[] _structure
    (* 4    (length _structure)) 4
    ;; float32[] _inferences
    (* 4    (length _inferences)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32[] _structure
     (write-long (length _structure) s)
     (dotimes (i (length _structure))
       (write-long (elt _structure i) s)
       )
     ;; float32[] _inferences
     (write-long (length _inferences) s)
     (dotimes (i (length _inferences))
       (sys::poke (elt _inferences i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32[] _structure
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _structure (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _structure i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; float32[] _inferences
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _inferences (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _inferences i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get model_server::InferenceResults :md5sum-) "794290d6a80514eaeded7ed9f07c9d16")
(setf (get model_server::InferenceResults :datatype-) "model_server/InferenceResults")
(setf (get model_server::InferenceResults :definition-)
      "int32[] structure
float32[]   inferences

")



(provide :model_server/InferenceResults "794290d6a80514eaeded7ed9f07c9d16")



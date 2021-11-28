;; Auto-generated. Do not edit!


(when (boundp 'model_server::ModelRequest)
  (if (not (find-package "MODEL_SERVER"))
    (make-package "MODEL_SERVER"))
  (shadow 'ModelRequest (find-package "MODEL_SERVER")))
(unless (find-package "MODEL_SERVER::MODELREQUEST")
  (make-package "MODEL_SERVER::MODELREQUEST"))
(unless (find-package "MODEL_SERVER::MODELREQUESTREQUEST")
  (make-package "MODEL_SERVER::MODELREQUESTREQUEST"))
(unless (find-package "MODEL_SERVER::MODELREQUESTRESPONSE")
  (make-package "MODEL_SERVER::MODELREQUESTRESPONSE"))

(in-package "ROS")

(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))




(defclass model_server::ModelRequestRequest
  :super ros::object
  :slots (_model ))

(defmethod model_server::ModelRequestRequest
  (:init
   (&key
    ((:model __model) (instance std_msgs::String :init))
    )
   (send-super :init)
   (setq _model __model)
   self)
  (:model
   (&rest __model)
   (if (keywordp (car __model))
       (send* _model __model)
     (progn
       (if __model (setq _model (car __model)))
       _model)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/String _model
    (send _model :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/String _model
       (send _model :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/String _model
     (send _model :deserialize buf ptr-) (incf ptr- (send _model :serialization-length))
   ;;
   self)
  )

(defclass model_server::ModelRequestResponse
  :super ros::object
  :slots (_results ))

(defmethod model_server::ModelRequestResponse
  (:init
   (&key
    ((:results __results) (instance model_server::InferenceResults :init))
    )
   (send-super :init)
   (setq _results __results)
   self)
  (:results
   (&rest __results)
   (if (keywordp (car __results))
       (send* _results __results)
     (progn
       (if __results (setq _results (car __results)))
       _results)))
  (:serialization-length
   ()
   (+
    ;; model_server/InferenceResults _results
    (send _results :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; model_server/InferenceResults _results
       (send _results :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; model_server/InferenceResults _results
     (send _results :deserialize buf ptr-) (incf ptr- (send _results :serialization-length))
   ;;
   self)
  )

(defclass model_server::ModelRequest
  :super ros::object
  :slots ())

(setf (get model_server::ModelRequest :md5sum-) "84e67d26165d3d65758a56814ed143ba")
(setf (get model_server::ModelRequest :datatype-) "model_server/ModelRequest")
(setf (get model_server::ModelRequest :request) model_server::ModelRequestRequest)
(setf (get model_server::ModelRequest :response) model_server::ModelRequestResponse)

(defmethod model_server::ModelRequestRequest
  (:response () (instance model_server::ModelRequestResponse :init)))

(setf (get model_server::ModelRequestRequest :md5sum-) "84e67d26165d3d65758a56814ed143ba")
(setf (get model_server::ModelRequestRequest :datatype-) "model_server/ModelRequestRequest")
(setf (get model_server::ModelRequestRequest :definition-)
      "std_msgs/String model

================================================================================
MSG: std_msgs/String
string data
---
InferenceResults    results


================================================================================
MSG: model_server/InferenceResults
int32[] structure
float32[]   inferences
")

(setf (get model_server::ModelRequestResponse :md5sum-) "84e67d26165d3d65758a56814ed143ba")
(setf (get model_server::ModelRequestResponse :datatype-) "model_server/ModelRequestResponse")
(setf (get model_server::ModelRequestResponse :definition-)
      "std_msgs/String model

================================================================================
MSG: std_msgs/String
string data
---
InferenceResults    results


================================================================================
MSG: model_server/InferenceResults
int32[] structure
float32[]   inferences
")



(provide :model_server/ModelRequest "84e67d26165d3d65758a56814ed143ba")



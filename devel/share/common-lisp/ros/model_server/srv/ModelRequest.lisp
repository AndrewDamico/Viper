; Auto-generated. Do not edit!


(cl:in-package model_server-srv)


;//! \htmlinclude ModelRequest-request.msg.html

(cl:defclass <ModelRequest-request> (roslisp-msg-protocol:ros-message)
  ((model
    :reader model
    :initarg :model
    :type std_msgs-msg:String
    :initform (cl:make-instance 'std_msgs-msg:String)))
)

(cl:defclass ModelRequest-request (<ModelRequest-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ModelRequest-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ModelRequest-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name model_server-srv:<ModelRequest-request> is deprecated: use model_server-srv:ModelRequest-request instead.")))

(cl:ensure-generic-function 'model-val :lambda-list '(m))
(cl:defmethod model-val ((m <ModelRequest-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader model_server-srv:model-val is deprecated.  Use model_server-srv:model instead.")
  (model m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ModelRequest-request>) ostream)
  "Serializes a message object of type '<ModelRequest-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'model) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ModelRequest-request>) istream)
  "Deserializes a message object of type '<ModelRequest-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'model) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ModelRequest-request>)))
  "Returns string type for a service object of type '<ModelRequest-request>"
  "model_server/ModelRequestRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ModelRequest-request)))
  "Returns string type for a service object of type 'ModelRequest-request"
  "model_server/ModelRequestRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ModelRequest-request>)))
  "Returns md5sum for a message object of type '<ModelRequest-request>"
  "84e67d26165d3d65758a56814ed143ba")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ModelRequest-request)))
  "Returns md5sum for a message object of type 'ModelRequest-request"
  "84e67d26165d3d65758a56814ed143ba")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ModelRequest-request>)))
  "Returns full string definition for message of type '<ModelRequest-request>"
  (cl:format cl:nil "std_msgs/String model~%~%================================================================================~%MSG: std_msgs/String~%string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ModelRequest-request)))
  "Returns full string definition for message of type 'ModelRequest-request"
  (cl:format cl:nil "std_msgs/String model~%~%================================================================================~%MSG: std_msgs/String~%string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ModelRequest-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'model))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ModelRequest-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ModelRequest-request
    (cl:cons ':model (model msg))
))
;//! \htmlinclude ModelRequest-response.msg.html

(cl:defclass <ModelRequest-response> (roslisp-msg-protocol:ros-message)
  ((results
    :reader results
    :initarg :results
    :type model_server-msg:InferenceResults
    :initform (cl:make-instance 'model_server-msg:InferenceResults)))
)

(cl:defclass ModelRequest-response (<ModelRequest-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ModelRequest-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ModelRequest-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name model_server-srv:<ModelRequest-response> is deprecated: use model_server-srv:ModelRequest-response instead.")))

(cl:ensure-generic-function 'results-val :lambda-list '(m))
(cl:defmethod results-val ((m <ModelRequest-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader model_server-srv:results-val is deprecated.  Use model_server-srv:results instead.")
  (results m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ModelRequest-response>) ostream)
  "Serializes a message object of type '<ModelRequest-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'results) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ModelRequest-response>) istream)
  "Deserializes a message object of type '<ModelRequest-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'results) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ModelRequest-response>)))
  "Returns string type for a service object of type '<ModelRequest-response>"
  "model_server/ModelRequestResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ModelRequest-response)))
  "Returns string type for a service object of type 'ModelRequest-response"
  "model_server/ModelRequestResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ModelRequest-response>)))
  "Returns md5sum for a message object of type '<ModelRequest-response>"
  "84e67d26165d3d65758a56814ed143ba")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ModelRequest-response)))
  "Returns md5sum for a message object of type 'ModelRequest-response"
  "84e67d26165d3d65758a56814ed143ba")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ModelRequest-response>)))
  "Returns full string definition for message of type '<ModelRequest-response>"
  (cl:format cl:nil "InferenceResults    results~%~%~%================================================================================~%MSG: model_server/InferenceResults~%int32[] structure~%float32[]   inferences~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ModelRequest-response)))
  "Returns full string definition for message of type 'ModelRequest-response"
  (cl:format cl:nil "InferenceResults    results~%~%~%================================================================================~%MSG: model_server/InferenceResults~%int32[] structure~%float32[]   inferences~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ModelRequest-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'results))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ModelRequest-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ModelRequest-response
    (cl:cons ':results (results msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ModelRequest)))
  'ModelRequest-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ModelRequest)))
  'ModelRequest-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ModelRequest)))
  "Returns string type for a service object of type '<ModelRequest>"
  "model_server/ModelRequest")
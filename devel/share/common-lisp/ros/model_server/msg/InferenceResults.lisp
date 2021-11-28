; Auto-generated. Do not edit!


(cl:in-package model_server-msg)


;//! \htmlinclude InferenceResults.msg.html

(cl:defclass <InferenceResults> (roslisp-msg-protocol:ros-message)
  ((structure
    :reader structure
    :initarg :structure
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0))
   (inferences
    :reader inferences
    :initarg :inferences
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass InferenceResults (<InferenceResults>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <InferenceResults>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'InferenceResults)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name model_server-msg:<InferenceResults> is deprecated: use model_server-msg:InferenceResults instead.")))

(cl:ensure-generic-function 'structure-val :lambda-list '(m))
(cl:defmethod structure-val ((m <InferenceResults>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader model_server-msg:structure-val is deprecated.  Use model_server-msg:structure instead.")
  (structure m))

(cl:ensure-generic-function 'inferences-val :lambda-list '(m))
(cl:defmethod inferences-val ((m <InferenceResults>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader model_server-msg:inferences-val is deprecated.  Use model_server-msg:inferences instead.")
  (inferences m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <InferenceResults>) ostream)
  "Serializes a message object of type '<InferenceResults>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'structure))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'structure))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'inferences))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'inferences))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <InferenceResults>) istream)
  "Deserializes a message object of type '<InferenceResults>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'structure) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'structure)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296)))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'inferences) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'inferences)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<InferenceResults>)))
  "Returns string type for a message object of type '<InferenceResults>"
  "model_server/InferenceResults")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'InferenceResults)))
  "Returns string type for a message object of type 'InferenceResults"
  "model_server/InferenceResults")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<InferenceResults>)))
  "Returns md5sum for a message object of type '<InferenceResults>"
  "794290d6a80514eaeded7ed9f07c9d16")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'InferenceResults)))
  "Returns md5sum for a message object of type 'InferenceResults"
  "794290d6a80514eaeded7ed9f07c9d16")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<InferenceResults>)))
  "Returns full string definition for message of type '<InferenceResults>"
  (cl:format cl:nil "int32[] structure~%float32[]   inferences~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'InferenceResults)))
  "Returns full string definition for message of type 'InferenceResults"
  (cl:format cl:nil "int32[] structure~%float32[]   inferences~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <InferenceResults>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'structure) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'inferences) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <InferenceResults>))
  "Converts a ROS message object to a list"
  (cl:list 'InferenceResults
    (cl:cons ':structure (structure msg))
    (cl:cons ':inferences (inferences msg))
))

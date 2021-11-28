; Auto-generated. Do not edit!


(cl:in-package model_server-srv)


;//! \htmlinclude ImageRequest-request.msg.html

(cl:defclass <ImageRequest-request> (roslisp-msg-protocol:ros-message)
  ((image
    :reader image
    :initarg :image
    :type sensor_msgs-msg:Image
    :initform (cl:make-instance 'sensor_msgs-msg:Image)))
)

(cl:defclass ImageRequest-request (<ImageRequest-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ImageRequest-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ImageRequest-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name model_server-srv:<ImageRequest-request> is deprecated: use model_server-srv:ImageRequest-request instead.")))

(cl:ensure-generic-function 'image-val :lambda-list '(m))
(cl:defmethod image-val ((m <ImageRequest-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader model_server-srv:image-val is deprecated.  Use model_server-srv:image instead.")
  (image m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ImageRequest-request>) ostream)
  "Serializes a message object of type '<ImageRequest-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'image) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ImageRequest-request>) istream)
  "Deserializes a message object of type '<ImageRequest-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'image) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ImageRequest-request>)))
  "Returns string type for a service object of type '<ImageRequest-request>"
  "model_server/ImageRequestRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ImageRequest-request)))
  "Returns string type for a service object of type 'ImageRequest-request"
  "model_server/ImageRequestRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ImageRequest-request>)))
  "Returns md5sum for a message object of type '<ImageRequest-request>"
  "fba05cbe092f5fd7d13b6a12503ed6ac")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ImageRequest-request)))
  "Returns md5sum for a message object of type 'ImageRequest-request"
  "fba05cbe092f5fd7d13b6a12503ed6ac")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ImageRequest-request>)))
  "Returns full string definition for message of type '<ImageRequest-request>"
  (cl:format cl:nil "sensor_msgs/Image   image~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of camera~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ImageRequest-request)))
  "Returns full string definition for message of type 'ImageRequest-request"
  (cl:format cl:nil "sensor_msgs/Image   image~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of camera~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ImageRequest-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'image))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ImageRequest-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ImageRequest-request
    (cl:cons ':image (image msg))
))
;//! \htmlinclude ImageRequest-response.msg.html

(cl:defclass <ImageRequest-response> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type std_msgs-msg:Bool
    :initform (cl:make-instance 'std_msgs-msg:Bool)))
)

(cl:defclass ImageRequest-response (<ImageRequest-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ImageRequest-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ImageRequest-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name model_server-srv:<ImageRequest-response> is deprecated: use model_server-srv:ImageRequest-response instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <ImageRequest-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader model_server-srv:status-val is deprecated.  Use model_server-srv:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ImageRequest-response>) ostream)
  "Serializes a message object of type '<ImageRequest-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'status) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ImageRequest-response>) istream)
  "Deserializes a message object of type '<ImageRequest-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'status) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ImageRequest-response>)))
  "Returns string type for a service object of type '<ImageRequest-response>"
  "model_server/ImageRequestResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ImageRequest-response)))
  "Returns string type for a service object of type 'ImageRequest-response"
  "model_server/ImageRequestResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ImageRequest-response>)))
  "Returns md5sum for a message object of type '<ImageRequest-response>"
  "fba05cbe092f5fd7d13b6a12503ed6ac")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ImageRequest-response)))
  "Returns md5sum for a message object of type 'ImageRequest-response"
  "fba05cbe092f5fd7d13b6a12503ed6ac")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ImageRequest-response>)))
  "Returns full string definition for message of type '<ImageRequest-response>"
  (cl:format cl:nil "std_msgs/Bool   status~%~%~%================================================================================~%MSG: std_msgs/Bool~%bool data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ImageRequest-response)))
  "Returns full string definition for message of type 'ImageRequest-response"
  (cl:format cl:nil "std_msgs/Bool   status~%~%~%================================================================================~%MSG: std_msgs/Bool~%bool data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ImageRequest-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'status))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ImageRequest-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ImageRequest-response
    (cl:cons ':status (status msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ImageRequest)))
  'ImageRequest-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ImageRequest)))
  'ImageRequest-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ImageRequest)))
  "Returns string type for a service object of type '<ImageRequest>"
  "model_server/ImageRequest")
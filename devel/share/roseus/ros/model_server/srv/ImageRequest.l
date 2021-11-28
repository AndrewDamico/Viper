;; Auto-generated. Do not edit!


(when (boundp 'model_server::ImageRequest)
  (if (not (find-package "MODEL_SERVER"))
    (make-package "MODEL_SERVER"))
  (shadow 'ImageRequest (find-package "MODEL_SERVER")))
(unless (find-package "MODEL_SERVER::IMAGEREQUEST")
  (make-package "MODEL_SERVER::IMAGEREQUEST"))
(unless (find-package "MODEL_SERVER::IMAGEREQUESTREQUEST")
  (make-package "MODEL_SERVER::IMAGEREQUESTREQUEST"))
(unless (find-package "MODEL_SERVER::IMAGEREQUESTRESPONSE")
  (make-package "MODEL_SERVER::IMAGEREQUESTRESPONSE"))

(in-package "ROS")

(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))


(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass model_server::ImageRequestRequest
  :super ros::object
  :slots (_image ))

(defmethod model_server::ImageRequestRequest
  (:init
   (&key
    ((:image __image) (instance sensor_msgs::Image :init))
    )
   (send-super :init)
   (setq _image __image)
   self)
  (:image
   (&rest __image)
   (if (keywordp (car __image))
       (send* _image __image)
     (progn
       (if __image (setq _image (car __image)))
       _image)))
  (:serialization-length
   ()
   (+
    ;; sensor_msgs/Image _image
    (send _image :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; sensor_msgs/Image _image
       (send _image :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; sensor_msgs/Image _image
     (send _image :deserialize buf ptr-) (incf ptr- (send _image :serialization-length))
   ;;
   self)
  )

(defclass model_server::ImageRequestResponse
  :super ros::object
  :slots (_status ))

(defmethod model_server::ImageRequestResponse
  (:init
   (&key
    ((:status __status) (instance std_msgs::Bool :init))
    )
   (send-super :init)
   (setq _status __status)
   self)
  (:status
   (&rest __status)
   (if (keywordp (car __status))
       (send* _status __status)
     (progn
       (if __status (setq _status (car __status)))
       _status)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Bool _status
    (send _status :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Bool _status
       (send _status :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Bool _status
     (send _status :deserialize buf ptr-) (incf ptr- (send _status :serialization-length))
   ;;
   self)
  )

(defclass model_server::ImageRequest
  :super ros::object
  :slots ())

(setf (get model_server::ImageRequest :md5sum-) "fba05cbe092f5fd7d13b6a12503ed6ac")
(setf (get model_server::ImageRequest :datatype-) "model_server/ImageRequest")
(setf (get model_server::ImageRequest :request) model_server::ImageRequestRequest)
(setf (get model_server::ImageRequest :response) model_server::ImageRequestResponse)

(defmethod model_server::ImageRequestRequest
  (:response () (instance model_server::ImageRequestResponse :init)))

(setf (get model_server::ImageRequestRequest :md5sum-) "fba05cbe092f5fd7d13b6a12503ed6ac")
(setf (get model_server::ImageRequestRequest :datatype-) "model_server/ImageRequestRequest")
(setf (get model_server::ImageRequestRequest :definition-)
      "sensor_msgs/Image   image

================================================================================
MSG: sensor_msgs/Image
# This message contains an uncompressed image
# (0, 0) is at top-left corner of image
#

Header header        # Header timestamp should be acquisition time of image
                     # Header frame_id should be optical frame of camera
                     # origin of frame should be optical center of camera
                     # +x should point to the right in the image
                     # +y should point down in the image
                     # +z should point into to plane of the image
                     # If the frame_id here and the frame_id of the CameraInfo
                     # message associated with the image conflict
                     # the behavior is undefined

uint32 height         # image height, that is, number of rows
uint32 width          # image width, that is, number of columns

# The legal values for encoding are in file src/image_encodings.cpp
# If you want to standardize a new string format, join
# ros-users@lists.sourceforge.net and send an email proposing a new encoding.

string encoding       # Encoding of pixels -- channel meaning, ordering, size
                      # taken from the list of strings in include/sensor_msgs/image_encodings.h

uint8 is_bigendian    # is this data bigendian?
uint32 step           # Full row length in bytes
uint8[] data          # actual matrix data, size is (step * rows)

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id
---
std_msgs/Bool   status


================================================================================
MSG: std_msgs/Bool
")

(setf (get model_server::ImageRequestResponse :md5sum-) "fba05cbe092f5fd7d13b6a12503ed6ac")
(setf (get model_server::ImageRequestResponse :datatype-) "model_server/ImageRequestResponse")
(setf (get model_server::ImageRequestResponse :definition-)
      "sensor_msgs/Image   image

================================================================================
MSG: sensor_msgs/Image
# This message contains an uncompressed image
# (0, 0) is at top-left corner of image
#

Header header        # Header timestamp should be acquisition time of image
                     # Header frame_id should be optical frame of camera
                     # origin of frame should be optical center of camera
                     # +x should point to the right in the image
                     # +y should point down in the image
                     # +z should point into to plane of the image
                     # If the frame_id here and the frame_id of the CameraInfo
                     # message associated with the image conflict
                     # the behavior is undefined

uint32 height         # image height, that is, number of rows
uint32 width          # image width, that is, number of columns

# The legal values for encoding are in file src/image_encodings.cpp
# If you want to standardize a new string format, join
# ros-users@lists.sourceforge.net and send an email proposing a new encoding.

string encoding       # Encoding of pixels -- channel meaning, ordering, size
                      # taken from the list of strings in include/sensor_msgs/image_encodings.h

uint8 is_bigendian    # is this data bigendian?
uint32 step           # Full row length in bytes
uint8[] data          # actual matrix data, size is (step * rows)

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id
---
std_msgs/Bool   status


================================================================================
MSG: std_msgs/Bool
")



(provide :model_server/ImageRequest "fba05cbe092f5fd7d13b6a12503ed6ac")



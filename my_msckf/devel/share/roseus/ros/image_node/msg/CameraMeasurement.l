;; Auto-generated. Do not edit!


(when (boundp 'image_node::CameraMeasurement)
  (if (not (find-package "IMAGE_NODE"))
    (make-package "IMAGE_NODE"))
  (shadow 'CameraMeasurement (find-package "IMAGE_NODE")))
(unless (find-package "IMAGE_NODE::CAMERAMEASUREMENT")
  (make-package "IMAGE_NODE::CAMERAMEASUREMENT"))

(in-package "ROS")
;;//! \htmlinclude CameraMeasurement.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass image_node::CameraMeasurement
  :super ros::object
  :slots (_header _features ))

(defmethod image_node::CameraMeasurement
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:features __features) (let (r) (dotimes (i 0) (push (instance image_node::FeatureMeasurement :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _features __features)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:features
   (&rest __features)
   (if (keywordp (car __features))
       (send* _features __features)
     (progn
       (if __features (setq _features (car __features)))
       _features)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; image_node/FeatureMeasurement[] _features
    (apply #'+ (send-all _features :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; image_node/FeatureMeasurement[] _features
     (write-long (length _features) s)
     (dolist (elem _features)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; image_node/FeatureMeasurement[] _features
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _features (let (r) (dotimes (i n) (push (instance image_node::FeatureMeasurement :init) r)) r))
     (dolist (elem- _features)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get image_node::CameraMeasurement :md5sum-) "03c4d3aa5670b44cd45362d0e5a1400a")
(setf (get image_node::CameraMeasurement :datatype-) "image_node/CameraMeasurement")
(setf (get image_node::CameraMeasurement :definition-)
      "std_msgs/Header header
# All features on the current image,
# including tracked ones and newly detected ones.
FeatureMeasurement[] features

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

================================================================================
MSG: image_node/FeatureMeasurement
uint64 id
# Normalized feature coordinates (with identity intrinsic matrix)
uint64 lifetime
float64 u0 # horizontal coordinate in cam0
float64 v0 # vertical coordinate in cam0
float64 u1 # horizontal coordinate in cam0
float64 v1 # vertical coordinate in cam0
")



(provide :image_node/CameraMeasurement "03c4d3aa5670b44cd45362d0e5a1400a")



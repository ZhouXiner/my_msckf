;; Auto-generated. Do not edit!


(when (boundp 'image_node::FeatureMeasurement)
  (if (not (find-package "IMAGE_NODE"))
    (make-package "IMAGE_NODE"))
  (shadow 'FeatureMeasurement (find-package "IMAGE_NODE")))
(unless (find-package "IMAGE_NODE::FEATUREMEASUREMENT")
  (make-package "IMAGE_NODE::FEATUREMEASUREMENT"))

(in-package "ROS")
;;//! \htmlinclude FeatureMeasurement.msg.html


(defclass image_node::FeatureMeasurement
  :super ros::object
  :slots (_id _lifetime _u0 _v0 _u1 _v1 ))

(defmethod image_node::FeatureMeasurement
  (:init
   (&key
    ((:id __id) 0)
    ((:lifetime __lifetime) 0)
    ((:u0 __u0) 0.0)
    ((:v0 __v0) 0.0)
    ((:u1 __u1) 0.0)
    ((:v1 __v1) 0.0)
    )
   (send-super :init)
   (setq _id (round __id))
   (setq _lifetime (round __lifetime))
   (setq _u0 (float __u0))
   (setq _v0 (float __v0))
   (setq _u1 (float __u1))
   (setq _v1 (float __v1))
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:lifetime
   (&optional __lifetime)
   (if __lifetime (setq _lifetime __lifetime)) _lifetime)
  (:u0
   (&optional __u0)
   (if __u0 (setq _u0 __u0)) _u0)
  (:v0
   (&optional __v0)
   (if __v0 (setq _v0 __v0)) _v0)
  (:u1
   (&optional __u1)
   (if __u1 (setq _u1 __u1)) _u1)
  (:v1
   (&optional __v1)
   (if __v1 (setq _v1 __v1)) _v1)
  (:serialization-length
   ()
   (+
    ;; uint64 _id
    8
    ;; uint64 _lifetime
    8
    ;; float64 _u0
    8
    ;; float64 _v0
    8
    ;; float64 _u1
    8
    ;; float64 _v1
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint64 _id
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _id (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _id) (= (length (_id . bv)) 2)) ;; bignum
              (write-long (ash (elt (_id . bv) 0) 0) s)
              (write-long (ash (elt (_id . bv) 1) -1) s))
             ((and (class _id) (= (length (_id . bv)) 1)) ;; big1
              (write-long (elt (_id . bv) 0) s)
              (write-long (if (>= _id 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _id s)(write-long (if (>= _id 0) 0 #xffffffff) s)))
     ;; uint64 _lifetime
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _lifetime (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _lifetime) (= (length (_lifetime . bv)) 2)) ;; bignum
              (write-long (ash (elt (_lifetime . bv) 0) 0) s)
              (write-long (ash (elt (_lifetime . bv) 1) -1) s))
             ((and (class _lifetime) (= (length (_lifetime . bv)) 1)) ;; big1
              (write-long (elt (_lifetime . bv) 0) s)
              (write-long (if (>= _lifetime 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _lifetime s)(write-long (if (>= _lifetime 0) 0 #xffffffff) s)))
     ;; float64 _u0
       (sys::poke _u0 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _v0
       (sys::poke _v0 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _u1
       (sys::poke _u1 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _v1
       (sys::poke _v1 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint64 _id
     
#+(or :alpha :irix6 :x86_64)
      (setf _id (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _id (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; uint64 _lifetime
     
#+(or :alpha :irix6 :x86_64)
      (setf _lifetime (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _lifetime (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; float64 _u0
     (setq _u0 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _v0
     (setq _v0 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _u1
     (setq _u1 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _v1
     (setq _v1 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get image_node::FeatureMeasurement :md5sum-) "b3f08e286803c7e02277cee87e2f026a")
(setf (get image_node::FeatureMeasurement :datatype-) "image_node/FeatureMeasurement")
(setf (get image_node::FeatureMeasurement :definition-)
      "uint64 id
# Normalized feature coordinates (with identity intrinsic matrix)
uint64 lifetime
float64 u0 # horizontal coordinate in cam0
float64 v0 # vertical coordinate in cam0
float64 u1 # horizontal coordinate in cam0
float64 v1 # vertical coordinate in cam0
")



(provide :image_node/FeatureMeasurement "b3f08e286803c7e02277cee87e2f026a")



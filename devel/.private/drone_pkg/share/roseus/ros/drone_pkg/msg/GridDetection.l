;; Auto-generated. Do not edit!


(when (boundp 'drone_pkg::GridDetection)
  (if (not (find-package "DRONE_PKG"))
    (make-package "DRONE_PKG"))
  (shadow 'GridDetection (find-package "DRONE_PKG")))
(unless (find-package "DRONE_PKG::GRIDDETECTION")
  (make-package "DRONE_PKG::GRIDDETECTION"))

(in-package "ROS")
;;//! \htmlinclude GridDetection.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass drone_pkg::GridDetection
  :super ros::object
  :slots (_header _row1 _row2 _row3 ))

(defmethod drone_pkg::GridDetection
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:row1 __row1) (make-array 0 :initial-element 0 :element-type :integer))
    ((:row2 __row2) (make-array 0 :initial-element 0 :element-type :integer))
    ((:row3 __row3) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _header __header)
   (setq _row1 __row1)
   (setq _row2 __row2)
   (setq _row3 __row3)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:row1
   (&optional __row1)
   (if __row1 (setq _row1 __row1)) _row1)
  (:row2
   (&optional __row2)
   (if __row2 (setq _row2 __row2)) _row2)
  (:row3
   (&optional __row3)
   (if __row3 (setq _row3 __row3)) _row3)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int32[] _row1
    (* 4    (length _row1)) 4
    ;; int32[] _row2
    (* 4    (length _row2)) 4
    ;; int32[] _row3
    (* 4    (length _row3)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int32[] _row1
     (write-long (length _row1) s)
     (dotimes (i (length _row1))
       (write-long (elt _row1 i) s)
       )
     ;; int32[] _row2
     (write-long (length _row2) s)
     (dotimes (i (length _row2))
       (write-long (elt _row2 i) s)
       )
     ;; int32[] _row3
     (write-long (length _row3) s)
     (dotimes (i (length _row3))
       (write-long (elt _row3 i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int32[] _row1
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _row1 (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _row1 i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; int32[] _row2
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _row2 (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _row2 i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; int32[] _row3
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _row3 (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _row3 i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get drone_pkg::GridDetection :md5sum-) "5913507622e805e6fa0e2a5ae80a4232")
(setf (get drone_pkg::GridDetection :datatype-) "drone_pkg/GridDetection")
(setf (get drone_pkg::GridDetection :definition-)
      "std_msgs/Header header
int32[] row1
int32[] row2
int32[] row3


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

")



(provide :drone_pkg/GridDetection "5913507622e805e6fa0e2a5ae80a4232")



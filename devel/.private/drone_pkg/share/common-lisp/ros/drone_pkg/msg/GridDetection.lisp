; Auto-generated. Do not edit!


(cl:in-package drone_pkg-msg)


;//! \htmlinclude GridDetection.msg.html

(cl:defclass <GridDetection> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (row1
    :reader row1
    :initarg :row1
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0))
   (row2
    :reader row2
    :initarg :row2
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0))
   (row3
    :reader row3
    :initarg :row3
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0)))
)

(cl:defclass GridDetection (<GridDetection>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GridDetection>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GridDetection)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name drone_pkg-msg:<GridDetection> is deprecated: use drone_pkg-msg:GridDetection instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <GridDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader drone_pkg-msg:header-val is deprecated.  Use drone_pkg-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'row1-val :lambda-list '(m))
(cl:defmethod row1-val ((m <GridDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader drone_pkg-msg:row1-val is deprecated.  Use drone_pkg-msg:row1 instead.")
  (row1 m))

(cl:ensure-generic-function 'row2-val :lambda-list '(m))
(cl:defmethod row2-val ((m <GridDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader drone_pkg-msg:row2-val is deprecated.  Use drone_pkg-msg:row2 instead.")
  (row2 m))

(cl:ensure-generic-function 'row3-val :lambda-list '(m))
(cl:defmethod row3-val ((m <GridDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader drone_pkg-msg:row3-val is deprecated.  Use drone_pkg-msg:row3 instead.")
  (row3 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GridDetection>) ostream)
  "Serializes a message object of type '<GridDetection>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'row1))))
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
   (cl:slot-value msg 'row1))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'row2))))
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
   (cl:slot-value msg 'row2))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'row3))))
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
   (cl:slot-value msg 'row3))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GridDetection>) istream)
  "Deserializes a message object of type '<GridDetection>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'row1) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'row1)))
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
  (cl:setf (cl:slot-value msg 'row2) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'row2)))
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
  (cl:setf (cl:slot-value msg 'row3) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'row3)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296)))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GridDetection>)))
  "Returns string type for a message object of type '<GridDetection>"
  "drone_pkg/GridDetection")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GridDetection)))
  "Returns string type for a message object of type 'GridDetection"
  "drone_pkg/GridDetection")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GridDetection>)))
  "Returns md5sum for a message object of type '<GridDetection>"
  "5913507622e805e6fa0e2a5ae80a4232")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GridDetection)))
  "Returns md5sum for a message object of type 'GridDetection"
  "5913507622e805e6fa0e2a5ae80a4232")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GridDetection>)))
  "Returns full string definition for message of type '<GridDetection>"
  (cl:format cl:nil "std_msgs/Header header~%int32[] row1~%int32[] row2~%int32[] row3~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GridDetection)))
  "Returns full string definition for message of type 'GridDetection"
  (cl:format cl:nil "std_msgs/Header header~%int32[] row1~%int32[] row2~%int32[] row3~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GridDetection>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'row1) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'row2) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'row3) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GridDetection>))
  "Converts a ROS message object to a list"
  (cl:list 'GridDetection
    (cl:cons ':header (header msg))
    (cl:cons ':row1 (row1 msg))
    (cl:cons ':row2 (row2 msg))
    (cl:cons ':row3 (row3 msg))
))

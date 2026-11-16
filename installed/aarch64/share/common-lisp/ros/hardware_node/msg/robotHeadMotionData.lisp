; Auto-generated. Do not edit!


(cl:in-package hardware_node-msg)


;//! \htmlinclude robotHeadMotionData.msg.html

(cl:defclass <robotHeadMotionData> (roslisp-msg-protocol:ros-message)
  ((target_position
    :reader target_position
    :initarg :target_position
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0)))
)

(cl:defclass robotHeadMotionData (<robotHeadMotionData>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <robotHeadMotionData>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'robotHeadMotionData)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hardware_node-msg:<robotHeadMotionData> is deprecated: use hardware_node-msg:robotHeadMotionData instead.")))

(cl:ensure-generic-function 'target_position-val :lambda-list '(m))
(cl:defmethod target_position-val ((m <robotHeadMotionData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hardware_node-msg:target_position-val is deprecated.  Use hardware_node-msg:target_position instead.")
  (target_position m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <robotHeadMotionData>) ostream)
  "Serializes a message object of type '<robotHeadMotionData>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'target_position))))
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
   (cl:slot-value msg 'target_position))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <robotHeadMotionData>) istream)
  "Deserializes a message object of type '<robotHeadMotionData>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'target_position) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'target_position)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296)))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<robotHeadMotionData>)))
  "Returns string type for a message object of type '<robotHeadMotionData>"
  "hardware_node/robotHeadMotionData")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'robotHeadMotionData)))
  "Returns string type for a message object of type 'robotHeadMotionData"
  "hardware_node/robotHeadMotionData")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<robotHeadMotionData>)))
  "Returns md5sum for a message object of type '<robotHeadMotionData>"
  "84c8c0833844df50ca703c90b47affbc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'robotHeadMotionData)))
  "Returns md5sum for a message object of type 'robotHeadMotionData"
  "84c8c0833844df50ca703c90b47affbc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<robotHeadMotionData>)))
  "Returns full string definition for message of type '<robotHeadMotionData>"
  (cl:format cl:nil "# 机器人头部电机位置 [0, 0]~%int32[] target_position~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'robotHeadMotionData)))
  "Returns full string definition for message of type 'robotHeadMotionData"
  (cl:format cl:nil "# 机器人头部电机位置 [0, 0]~%int32[] target_position~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <robotHeadMotionData>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'target_position) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <robotHeadMotionData>))
  "Converts a ROS message object to a list"
  (cl:list 'robotHeadMotionData
    (cl:cons ':target_position (target_position msg))
))

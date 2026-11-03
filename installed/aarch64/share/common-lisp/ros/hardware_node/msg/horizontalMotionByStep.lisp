; Auto-generated. Do not edit!


(cl:in-package hardware_node-msg)


;//! \htmlinclude horizontalMotionByStep.msg.html

(cl:defclass <horizontalMotionByStep> (roslisp-msg-protocol:ros-message)
  ((step
    :reader step
    :initarg :step
    :type cl:integer
    :initform 0))
)

(cl:defclass horizontalMotionByStep (<horizontalMotionByStep>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <horizontalMotionByStep>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'horizontalMotionByStep)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hardware_node-msg:<horizontalMotionByStep> is deprecated: use hardware_node-msg:horizontalMotionByStep instead.")))

(cl:ensure-generic-function 'step-val :lambda-list '(m))
(cl:defmethod step-val ((m <horizontalMotionByStep>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hardware_node-msg:step-val is deprecated.  Use hardware_node-msg:step instead.")
  (step m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <horizontalMotionByStep>) ostream)
  "Serializes a message object of type '<horizontalMotionByStep>"
  (cl:let* ((signed (cl:slot-value msg 'step)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <horizontalMotionByStep>) istream)
  "Deserializes a message object of type '<horizontalMotionByStep>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'step) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<horizontalMotionByStep>)))
  "Returns string type for a message object of type '<horizontalMotionByStep>"
  "hardware_node/horizontalMotionByStep")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'horizontalMotionByStep)))
  "Returns string type for a message object of type 'horizontalMotionByStep"
  "hardware_node/horizontalMotionByStep")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<horizontalMotionByStep>)))
  "Returns md5sum for a message object of type '<horizontalMotionByStep>"
  "99174260c0c07917ce2b7a46302ab7a8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'horizontalMotionByStep)))
  "Returns md5sum for a message object of type 'horizontalMotionByStep"
  "99174260c0c07917ce2b7a46302ab7a8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<horizontalMotionByStep>)))
  "Returns full string definition for message of type '<horizontalMotionByStep>"
  (cl:format cl:nil "# 单步左右转动，传入值为转动的步长~%int32 step~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'horizontalMotionByStep)))
  "Returns full string definition for message of type 'horizontalMotionByStep"
  (cl:format cl:nil "# 单步左右转动，传入值为转动的步长~%int32 step~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <horizontalMotionByStep>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <horizontalMotionByStep>))
  "Converts a ROS message object to a list"
  (cl:list 'horizontalMotionByStep
    (cl:cons ':step (step msg))
))

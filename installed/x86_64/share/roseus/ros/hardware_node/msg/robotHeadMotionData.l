;; Auto-generated. Do not edit!


(when (boundp 'hardware_node::robotHeadMotionData)
  (if (not (find-package "HARDWARE_NODE"))
    (make-package "HARDWARE_NODE"))
  (shadow 'robotHeadMotionData (find-package "HARDWARE_NODE")))
(unless (find-package "HARDWARE_NODE::ROBOTHEADMOTIONDATA")
  (make-package "HARDWARE_NODE::ROBOTHEADMOTIONDATA"))

(in-package "ROS")
;;//! \htmlinclude robotHeadMotionData.msg.html


(defclass hardware_node::robotHeadMotionData
  :super ros::object
  :slots (_target_position ))

(defmethod hardware_node::robotHeadMotionData
  (:init
   (&key
    ((:target_position __target_position) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _target_position __target_position)
   self)
  (:target_position
   (&optional __target_position)
   (if __target_position (setq _target_position __target_position)) _target_position)
  (:serialization-length
   ()
   (+
    ;; int32[] _target_position
    (* 4    (length _target_position)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32[] _target_position
     (write-long (length _target_position) s)
     (dotimes (i (length _target_position))
       (write-long (elt _target_position i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32[] _target_position
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _target_position (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _target_position i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get hardware_node::robotHeadMotionData :md5sum-) "84c8c0833844df50ca703c90b47affbc")
(setf (get hardware_node::robotHeadMotionData :datatype-) "hardware_node/robotHeadMotionData")
(setf (get hardware_node::robotHeadMotionData :definition-)
      "# 机器人头部电机位置 [0, 0]
int32[] target_position
")



(provide :hardware_node/robotHeadMotionData "84c8c0833844df50ca703c90b47affbc")



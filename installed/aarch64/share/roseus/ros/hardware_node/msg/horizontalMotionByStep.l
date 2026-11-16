;; Auto-generated. Do not edit!


(when (boundp 'hardware_node::horizontalMotionByStep)
  (if (not (find-package "HARDWARE_NODE"))
    (make-package "HARDWARE_NODE"))
  (shadow 'horizontalMotionByStep (find-package "HARDWARE_NODE")))
(unless (find-package "HARDWARE_NODE::HORIZONTALMOTIONBYSTEP")
  (make-package "HARDWARE_NODE::HORIZONTALMOTIONBYSTEP"))

(in-package "ROS")
;;//! \htmlinclude horizontalMotionByStep.msg.html


(defclass hardware_node::horizontalMotionByStep
  :super ros::object
  :slots (_step ))

(defmethod hardware_node::horizontalMotionByStep
  (:init
   (&key
    ((:step __step) 0)
    )
   (send-super :init)
   (setq _step (round __step))
   self)
  (:step
   (&optional __step)
   (if __step (setq _step __step)) _step)
  (:serialization-length
   ()
   (+
    ;; int32 _step
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _step
       (write-long _step s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _step
     (setq _step (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get hardware_node::horizontalMotionByStep :md5sum-) "99174260c0c07917ce2b7a46302ab7a8")
(setf (get hardware_node::horizontalMotionByStep :datatype-) "hardware_node/horizontalMotionByStep")
(setf (get hardware_node::horizontalMotionByStep :definition-)
      "# 单步左右转动，传入值为转动的步长
int32 step
")



(provide :hardware_node/horizontalMotionByStep "99174260c0c07917ce2b7a46302ab7a8")



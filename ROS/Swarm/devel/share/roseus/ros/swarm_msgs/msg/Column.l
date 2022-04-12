;; Auto-generated. Do not edit!


(when (boundp 'swarm_msgs::Column)
  (if (not (find-package "SWARM_MSGS"))
    (make-package "SWARM_MSGS"))
  (shadow 'Column (find-package "SWARM_MSGS")))
(unless (find-package "SWARM_MSGS::COLUMN")
  (make-package "SWARM_MSGS::COLUMN"))

(in-package "ROS")
;;//! \htmlinclude Column.msg.html


(defclass swarm_msgs::Column
  :super ros::object
  :slots (_row ))

(defmethod swarm_msgs::Column
  (:init
   (&key
    ((:row __row) (let (r) (dotimes (i 10) (push (instance swarm_msgs::Square :init) r)) r))
    )
   (send-super :init)
   (setq _row __row)
   self)
  (:row
   (&rest __row)
   (if (keywordp (car __row))
       (send* _row __row)
     (progn
       (if __row (setq _row (car __row)))
       _row)))
  (:serialization-length
   ()
   (+
    ;; swarm_msgs/Square[10] _row
    (apply #'+ (send-all _row :serialization-length))
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; swarm_msgs/Square[10] _row
     (dolist (elem _row)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; swarm_msgs/Square[10] _row
   (dotimes (i 10)
     (send (elt _row i) :deserialize buf ptr-) (incf ptr- (send (elt _row i) :serialization-length))
     )
   ;;
   self)
  )

(setf (get swarm_msgs::Column :md5sum-) "67820cc83c22bec62d23e6552a65e66b")
(setf (get swarm_msgs::Column :datatype-) "swarm_msgs/Column")
(setf (get swarm_msgs::Column :definition-)
      "Square[10] row
================================================================================
MSG: swarm_msgs/Square
int8[5] pheromones
")



(provide :swarm_msgs/Column "67820cc83c22bec62d23e6552a65e66b")



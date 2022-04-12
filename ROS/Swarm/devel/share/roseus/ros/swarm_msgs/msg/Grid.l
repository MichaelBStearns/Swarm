;; Auto-generated. Do not edit!


(when (boundp 'swarm_msgs::Grid)
  (if (not (find-package "SWARM_MSGS"))
    (make-package "SWARM_MSGS"))
  (shadow 'Grid (find-package "SWARM_MSGS")))
(unless (find-package "SWARM_MSGS::GRID")
  (make-package "SWARM_MSGS::GRID"))

(in-package "ROS")
;;//! \htmlinclude Grid.msg.html


(defclass swarm_msgs::Grid
  :super ros::object
  :slots (_column ))

(defmethod swarm_msgs::Grid
  (:init
   (&key
    ((:column __column) (let (r) (dotimes (i 10) (push (instance swarm_msgs::Column :init) r)) r))
    )
   (send-super :init)
   (setq _column __column)
   self)
  (:column
   (&rest __column)
   (if (keywordp (car __column))
       (send* _column __column)
     (progn
       (if __column (setq _column (car __column)))
       _column)))
  (:serialization-length
   ()
   (+
    ;; swarm_msgs/Column[10] _column
    (apply #'+ (send-all _column :serialization-length))
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; swarm_msgs/Column[10] _column
     (dolist (elem _column)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; swarm_msgs/Column[10] _column
   (dotimes (i 10)
     (send (elt _column i) :deserialize buf ptr-) (incf ptr- (send (elt _column i) :serialization-length))
     )
   ;;
   self)
  )

(setf (get swarm_msgs::Grid :md5sum-) "c5b445e2bca73a30cc767bc84f07b7c1")
(setf (get swarm_msgs::Grid :datatype-) "swarm_msgs/Grid")
(setf (get swarm_msgs::Grid :definition-)
      "Column[10] column
================================================================================
MSG: swarm_msgs/Column
Square[10] row
================================================================================
MSG: swarm_msgs/Square
int8[5] pheromones
")



(provide :swarm_msgs/Grid "c5b445e2bca73a30cc767bc84f07b7c1")



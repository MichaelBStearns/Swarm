;; Auto-generated. Do not edit!


(when (boundp 'swarm_msgs::Square)
  (if (not (find-package "SWARM_MSGS"))
    (make-package "SWARM_MSGS"))
  (shadow 'Square (find-package "SWARM_MSGS")))
(unless (find-package "SWARM_MSGS::SQUARE")
  (make-package "SWARM_MSGS::SQUARE"))

(in-package "ROS")
;;//! \htmlinclude Square.msg.html


(defclass swarm_msgs::Square
  :super ros::object
  :slots (_pheromones ))

(defmethod swarm_msgs::Square
  (:init
   (&key
    ((:pheromones __pheromones) (make-array 5 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _pheromones __pheromones)
   self)
  (:pheromones
   (&optional __pheromones)
   (if __pheromones (setq _pheromones __pheromones)) _pheromones)
  (:serialization-length
   ()
   (+
    ;; int8[5] _pheromones
    (* 1    5)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8[5] _pheromones
     (dotimes (i 5)
       (write-byte (elt _pheromones i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8[5] _pheromones
   (dotimes (i (length _pheromones))
     (setf (elt _pheromones i) (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> (elt _pheromones i) 127) (setf (elt _pheromones i) (- (elt _pheromones i) 256)))
     )
   ;;
   self)
  )

(setf (get swarm_msgs::Square :md5sum-) "98b485e50aa2d6a27ada82a8008715c5")
(setf (get swarm_msgs::Square :datatype-) "swarm_msgs/Square")
(setf (get swarm_msgs::Square :definition-)
      "int8[5] pheromones
")



(provide :swarm_msgs/Square "98b485e50aa2d6a27ada82a8008715c5")



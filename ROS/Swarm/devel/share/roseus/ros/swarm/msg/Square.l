;; Auto-generated. Do not edit!


(when (boundp 'swarm::Square)
  (if (not (find-package "SWARM"))
    (make-package "SWARM"))
  (shadow 'Square (find-package "SWARM")))
(unless (find-package "SWARM::SQUARE")
  (make-package "SWARM::SQUARE"))

(in-package "ROS")
;;//! \htmlinclude Square.msg.html


(defclass swarm::Square
  :super ros::object
  :slots (_pheromones ))

(defmethod swarm::Square
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
    ;; int16[5] _pheromones
    (* 2    5)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int16[5] _pheromones
     (dotimes (i 5)
       (write-word (elt _pheromones i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int16[5] _pheromones
   (dotimes (i (length _pheromones))
     (setf (elt _pheromones i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     )
   ;;
   self)
  )

(setf (get swarm::Square :md5sum-) "981e6a473130398a846d4267fa69ed65")
(setf (get swarm::Square :datatype-) "swarm/Square")
(setf (get swarm::Square :definition-)
      "int16[5] pheromones
")



(provide :swarm/Square "981e6a473130398a846d4267fa69ed65")



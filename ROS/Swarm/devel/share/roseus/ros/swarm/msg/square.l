;; Auto-generated. Do not edit!


(when (boundp 'swarm::square)
  (if (not (find-package "SWARM"))
    (make-package "SWARM"))
  (shadow 'square (find-package "SWARM")))
(unless (find-package "SWARM::SQUARE")
  (make-package "SWARM::SQUARE"))

(in-package "ROS")
;;//! \htmlinclude square.msg.html


(defclass swarm::square
  :super ros::object
  :slots (_pheromones ))

(defmethod swarm::square
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
    ;; char[5] _pheromones
    (* 1    5)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; char[5] _pheromones
     (dotimes (i 5)
       (write-byte (elt _pheromones i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; char[5] _pheromones
   (dotimes (i (length _pheromones))
     (setf (elt _pheromones i) (sys::peek buf ptr- :char)) (incf ptr- 1)
     )
   ;;
   self)
  )

(setf (get swarm::square :md5sum-) "ad37cb41072571a31d12e95c135b1050")
(setf (get swarm::square :datatype-) "swarm/square")
(setf (get swarm::square :definition-)
      "char[5] pheromones
")



(provide :swarm/square "ad37cb41072571a31d12e95c135b1050")



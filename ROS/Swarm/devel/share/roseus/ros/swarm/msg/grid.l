;; Auto-generated. Do not edit!


(when (boundp 'swarm::grid)
  (if (not (find-package "SWARM"))
    (make-package "SWARM"))
  (shadow 'grid (find-package "SWARM")))
(unless (find-package "SWARM::GRID")
  (make-package "SWARM::GRID"))

(in-package "ROS")
;;//! \htmlinclude grid.msg.html


(defclass swarm::grid
  :super ros::object
  :slots (_box ))

(defmethod swarm::grid
  (:init
   (&key
    ((:box __box) (instance swarm::square :init))
    )
   (send-super :init)
   (setq _box __box)
   self)
  (:box
   (&rest __box)
   (if (keywordp (car __box))
       (send* _box __box)
     (progn
       (if __box (setq _box (car __box)))
       _box)))
  (:serialization-length
   ()
   (+
    ;; swarm/square _box
    (send _box :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; swarm/square _box
       (send _box :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; swarm/square _box
     (send _box :deserialize buf ptr-) (incf ptr- (send _box :serialization-length))
   ;;
   self)
  )

(setf (get swarm::grid :md5sum-) "32a280d72cb6e8c663e8b4d302b71970")
(setf (get swarm::grid :datatype-) "swarm/grid")
(setf (get swarm::grid :definition-)
      "square box
================================================================================
MSG: swarm/square
char[5] pheromones
")



(provide :swarm/grid "32a280d72cb6e8c663e8b4d302b71970")



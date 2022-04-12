;; Auto-generated. Do not edit!


(when (boundp 'swarm::column)
  (if (not (find-package "SWARM"))
    (make-package "SWARM"))
  (shadow 'column (find-package "SWARM")))
(unless (find-package "SWARM::COLUMN")
  (make-package "SWARM::COLUMN"))

(in-package "ROS")
;;//! \htmlinclude column.msg.html


(defclass swarm::column
  :super ros::object
  :slots (_row ))

(defmethod swarm::column
  (:init
   (&key
    ((:row __row) (let (r) (dotimes (i 100) (push (instance swarm::square :init) r)) r))
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
    ;; swarm/square[100] _row
    (apply #'+ (send-all _row :serialization-length))
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; swarm/square[100] _row
     (dolist (elem _row)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; swarm/square[100] _row
   (dotimes (i 100)
     (send (elt _row i) :deserialize buf ptr-) (incf ptr- (send (elt _row i) :serialization-length))
     )
   ;;
   self)
  )

(setf (get swarm::column :md5sum-) "3606c5dc7d55d80c61e5a77e52f55b69")
(setf (get swarm::column :datatype-) "swarm/column")
(setf (get swarm::column :definition-)
      "square[100] row
================================================================================
MSG: swarm/square
char[5] pheromones
")



(provide :swarm/column "3606c5dc7d55d80c61e5a77e52f55b69")



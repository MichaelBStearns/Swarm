;; Auto-generated. Do not edit!


(when (boundp 'swarm::Column)
  (if (not (find-package "SWARM"))
    (make-package "SWARM"))
  (shadow 'Column (find-package "SWARM")))
(unless (find-package "SWARM::COLUMN")
  (make-package "SWARM::COLUMN"))

(in-package "ROS")
;;//! \htmlinclude Column.msg.html


(defclass swarm::Column
  :super ros::object
  :slots (_row ))

(defmethod swarm::Column
  (:init
   (&key
    ((:row __row) (let (r) (dotimes (i 100) (push (instance swarm::Square :init) r)) r))
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
    ;; swarm/Square[100] _row
    (apply #'+ (send-all _row :serialization-length))
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; swarm/Square[100] _row
     (dolist (elem _row)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; swarm/Square[100] _row
   (dotimes (i 100)
     (send (elt _row i) :deserialize buf ptr-) (incf ptr- (send (elt _row i) :serialization-length))
     )
   ;;
   self)
  )

(setf (get swarm::Column :md5sum-) "ddf6a97d0cfbc524eaea9ad493826b1d")
(setf (get swarm::Column :datatype-) "swarm/Column")
(setf (get swarm::Column :definition-)
      "Square[100] row
================================================================================
MSG: swarm/Square
int16[5] pheromones
")



(provide :swarm/Column "ddf6a97d0cfbc524eaea9ad493826b1d")



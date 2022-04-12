;; Auto-generated. Do not edit!


(when (boundp 'swarm::Grid)
  (if (not (find-package "SWARM"))
    (make-package "SWARM"))
  (shadow 'Grid (find-package "SWARM")))
(unless (find-package "SWARM::GRID")
  (make-package "SWARM::GRID"))

(in-package "ROS")
;;//! \htmlinclude Grid.msg.html


(defclass swarm::Grid
  :super ros::object
  :slots (_column ))

(defmethod swarm::Grid
  (:init
   (&key
    ((:column __column) (let (r) (dotimes (i 100) (push (instance swarm::Column :init) r)) r))
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
    ;; swarm/Column[100] _column
    (apply #'+ (send-all _column :serialization-length))
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; swarm/Column[100] _column
     (dolist (elem _column)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; swarm/Column[100] _column
   (dotimes (i 100)
     (send (elt _column i) :deserialize buf ptr-) (incf ptr- (send (elt _column i) :serialization-length))
     )
   ;;
   self)
  )

(setf (get swarm::Grid :md5sum-) "243c44d4957c298207a8acfe78eede41")
(setf (get swarm::Grid :datatype-) "swarm/Grid")
(setf (get swarm::Grid :definition-)
      "Column[100] column
================================================================================
MSG: swarm/Column
Square[100] row
================================================================================
MSG: swarm/Square
int16[5] pheromones
")



(provide :swarm/Grid "243c44d4957c298207a8acfe78eede41")



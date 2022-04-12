; Auto-generated. Do not edit!


(cl:in-package swarm-msg)


;//! \htmlinclude grid.msg.html

(cl:defclass <grid> (roslisp-msg-protocol:ros-message)
  ((box
    :reader box
    :initarg :box
    :type swarm-msg:square
    :initform (cl:make-instance 'swarm-msg:square)))
)

(cl:defclass grid (<grid>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <grid>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'grid)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name swarm-msg:<grid> is deprecated: use swarm-msg:grid instead.")))

(cl:ensure-generic-function 'box-val :lambda-list '(m))
(cl:defmethod box-val ((m <grid>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader swarm-msg:box-val is deprecated.  Use swarm-msg:box instead.")
  (box m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <grid>) ostream)
  "Serializes a message object of type '<grid>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'box) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <grid>) istream)
  "Deserializes a message object of type '<grid>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'box) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<grid>)))
  "Returns string type for a message object of type '<grid>"
  "swarm/grid")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'grid)))
  "Returns string type for a message object of type 'grid"
  "swarm/grid")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<grid>)))
  "Returns md5sum for a message object of type '<grid>"
  "32a280d72cb6e8c663e8b4d302b71970")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'grid)))
  "Returns md5sum for a message object of type 'grid"
  "32a280d72cb6e8c663e8b4d302b71970")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<grid>)))
  "Returns full string definition for message of type '<grid>"
  (cl:format cl:nil "square box~%================================================================================~%MSG: swarm/square~%char[5] pheromones~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'grid)))
  "Returns full string definition for message of type 'grid"
  (cl:format cl:nil "square box~%================================================================================~%MSG: swarm/square~%char[5] pheromones~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <grid>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'box))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <grid>))
  "Converts a ROS message object to a list"
  (cl:list 'grid
    (cl:cons ':box (box msg))
))

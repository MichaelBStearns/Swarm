; Auto-generated. Do not edit!


(cl:in-package swarm-msg)


;//! \htmlinclude square.msg.html

(cl:defclass <square> (roslisp-msg-protocol:ros-message)
  ((pheromones
    :reader pheromones
    :initarg :pheromones
    :type (cl:vector cl:integer)
   :initform (cl:make-array 5 :element-type 'cl:integer :initial-element 0)))
)

(cl:defclass square (<square>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <square>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'square)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name swarm-msg:<square> is deprecated: use swarm-msg:square instead.")))

(cl:ensure-generic-function 'pheromones-val :lambda-list '(m))
(cl:defmethod pheromones-val ((m <square>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader swarm-msg:pheromones-val is deprecated.  Use swarm-msg:pheromones instead.")
  (pheromones m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <square>) ostream)
  "Serializes a message object of type '<square>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'pheromones))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <square>) istream)
  "Deserializes a message object of type '<square>"
  (cl:setf (cl:slot-value msg 'pheromones) (cl:make-array 5))
  (cl:let ((vals (cl:slot-value msg 'pheromones)))
    (cl:dotimes (i 5)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<square>)))
  "Returns string type for a message object of type '<square>"
  "swarm/square")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'square)))
  "Returns string type for a message object of type 'square"
  "swarm/square")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<square>)))
  "Returns md5sum for a message object of type '<square>"
  "ad37cb41072571a31d12e95c135b1050")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'square)))
  "Returns md5sum for a message object of type 'square"
  "ad37cb41072571a31d12e95c135b1050")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<square>)))
  "Returns full string definition for message of type '<square>"
  (cl:format cl:nil "char[5] pheromones~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'square)))
  "Returns full string definition for message of type 'square"
  (cl:format cl:nil "char[5] pheromones~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <square>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'pheromones) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <square>))
  "Converts a ROS message object to a list"
  (cl:list 'square
    (cl:cons ':pheromones (pheromones msg))
))

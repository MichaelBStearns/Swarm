; Auto-generated. Do not edit!


(cl:in-package swarm-msg)


;//! \htmlinclude Square.msg.html

(cl:defclass <Square> (roslisp-msg-protocol:ros-message)
  ((pheromones
    :reader pheromones
    :initarg :pheromones
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 5 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass Square (<Square>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Square>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Square)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name swarm-msg:<Square> is deprecated: use swarm-msg:Square instead.")))

(cl:ensure-generic-function 'pheromones-val :lambda-list '(m))
(cl:defmethod pheromones-val ((m <Square>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader swarm-msg:pheromones-val is deprecated.  Use swarm-msg:pheromones instead.")
  (pheromones m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Square>) ostream)
  "Serializes a message object of type '<Square>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    ))
   (cl:slot-value msg 'pheromones))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Square>) istream)
  "Deserializes a message object of type '<Square>"
  (cl:setf (cl:slot-value msg 'pheromones) (cl:make-array 5))
  (cl:let ((vals (cl:slot-value msg 'pheromones)))
    (cl:dotimes (i 5)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Square>)))
  "Returns string type for a message object of type '<Square>"
  "swarm/Square")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Square)))
  "Returns string type for a message object of type 'Square"
  "swarm/Square")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Square>)))
  "Returns md5sum for a message object of type '<Square>"
  "981e6a473130398a846d4267fa69ed65")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Square)))
  "Returns md5sum for a message object of type 'Square"
  "981e6a473130398a846d4267fa69ed65")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Square>)))
  "Returns full string definition for message of type '<Square>"
  (cl:format cl:nil "int16[5] pheromones~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Square)))
  "Returns full string definition for message of type 'Square"
  (cl:format cl:nil "int16[5] pheromones~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Square>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'pheromones) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Square>))
  "Converts a ROS message object to a list"
  (cl:list 'Square
    (cl:cons ':pheromones (pheromones msg))
))

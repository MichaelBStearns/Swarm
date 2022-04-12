; Auto-generated. Do not edit!


(cl:in-package swarm-msg)


;//! \htmlinclude column.msg.html

(cl:defclass <column> (roslisp-msg-protocol:ros-message)
  ((row
    :reader row
    :initarg :row
    :type (cl:vector swarm-msg:square)
   :initform (cl:make-array 100 :element-type 'swarm-msg:square :initial-element (cl:make-instance 'swarm-msg:square))))
)

(cl:defclass column (<column>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <column>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'column)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name swarm-msg:<column> is deprecated: use swarm-msg:column instead.")))

(cl:ensure-generic-function 'row-val :lambda-list '(m))
(cl:defmethod row-val ((m <column>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader swarm-msg:row-val is deprecated.  Use swarm-msg:row instead.")
  (row m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <column>) ostream)
  "Serializes a message object of type '<column>"
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'row))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <column>) istream)
  "Deserializes a message object of type '<column>"
  (cl:setf (cl:slot-value msg 'row) (cl:make-array 100))
  (cl:let ((vals (cl:slot-value msg 'row)))
    (cl:dotimes (i 100)
    (cl:setf (cl:aref vals i) (cl:make-instance 'swarm-msg:square))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<column>)))
  "Returns string type for a message object of type '<column>"
  "swarm/column")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'column)))
  "Returns string type for a message object of type 'column"
  "swarm/column")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<column>)))
  "Returns md5sum for a message object of type '<column>"
  "3606c5dc7d55d80c61e5a77e52f55b69")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'column)))
  "Returns md5sum for a message object of type 'column"
  "3606c5dc7d55d80c61e5a77e52f55b69")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<column>)))
  "Returns full string definition for message of type '<column>"
  (cl:format cl:nil "square[100] row~%================================================================================~%MSG: swarm/square~%char[5] pheromones~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'column)))
  "Returns full string definition for message of type 'column"
  (cl:format cl:nil "square[100] row~%================================================================================~%MSG: swarm/square~%char[5] pheromones~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <column>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'row) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <column>))
  "Converts a ROS message object to a list"
  (cl:list 'column
    (cl:cons ':row (row msg))
))

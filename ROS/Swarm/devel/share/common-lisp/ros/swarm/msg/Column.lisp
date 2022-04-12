; Auto-generated. Do not edit!


(cl:in-package swarm-msg)


;//! \htmlinclude Column.msg.html

(cl:defclass <Column> (roslisp-msg-protocol:ros-message)
  ((row
    :reader row
    :initarg :row
    :type (cl:vector swarm-msg:Square)
   :initform (cl:make-array 100 :element-type 'swarm-msg:Square :initial-element (cl:make-instance 'swarm-msg:Square))))
)

(cl:defclass Column (<Column>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Column>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Column)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name swarm-msg:<Column> is deprecated: use swarm-msg:Column instead.")))

(cl:ensure-generic-function 'row-val :lambda-list '(m))
(cl:defmethod row-val ((m <Column>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader swarm-msg:row-val is deprecated.  Use swarm-msg:row instead.")
  (row m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Column>) ostream)
  "Serializes a message object of type '<Column>"
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'row))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Column>) istream)
  "Deserializes a message object of type '<Column>"
  (cl:setf (cl:slot-value msg 'row) (cl:make-array 100))
  (cl:let ((vals (cl:slot-value msg 'row)))
    (cl:dotimes (i 100)
    (cl:setf (cl:aref vals i) (cl:make-instance 'swarm-msg:Square))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Column>)))
  "Returns string type for a message object of type '<Column>"
  "swarm/Column")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Column)))
  "Returns string type for a message object of type 'Column"
  "swarm/Column")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Column>)))
  "Returns md5sum for a message object of type '<Column>"
  "ddf6a97d0cfbc524eaea9ad493826b1d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Column)))
  "Returns md5sum for a message object of type 'Column"
  "ddf6a97d0cfbc524eaea9ad493826b1d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Column>)))
  "Returns full string definition for message of type '<Column>"
  (cl:format cl:nil "Square[100] row~%================================================================================~%MSG: swarm/Square~%int16[5] pheromones~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Column)))
  "Returns full string definition for message of type 'Column"
  (cl:format cl:nil "Square[100] row~%================================================================================~%MSG: swarm/Square~%int16[5] pheromones~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Column>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'row) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Column>))
  "Converts a ROS message object to a list"
  (cl:list 'Column
    (cl:cons ':row (row msg))
))

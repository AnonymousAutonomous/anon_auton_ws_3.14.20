; Auto-generated. Do not edit!


(cl:in-package eyes-msg)


;//! \htmlinclude Generic.msg.html

(cl:defclass <Generic> (roslisp-msg-protocol:ros-message)
  ((identifier
    :reader identifier
    :initarg :identifier
    :type cl:integer
    :initform 0)
   (left_forward
    :reader left_forward
    :initarg :left_forward
    :type cl:boolean
    :initform cl:nil)
   (right_forward
    :reader right_forward
    :initarg :right_forward
    :type cl:boolean
    :initform cl:nil)
   (left_speed
    :reader left_speed
    :initarg :left_speed
    :type cl:fixnum
    :initform 0)
   (right_speed
    :reader right_speed
    :initarg :right_speed
    :type cl:fixnum
    :initform 0)
   (timed
    :reader timed
    :initarg :timed
    :type cl:boolean
    :initform cl:nil)
   (duration
    :reader duration
    :initarg :duration
    :type cl:integer
    :initform 0))
)

(cl:defclass Generic (<Generic>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Generic>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Generic)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name eyes-msg:<Generic> is deprecated: use eyes-msg:Generic instead.")))

(cl:ensure-generic-function 'identifier-val :lambda-list '(m))
(cl:defmethod identifier-val ((m <Generic>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader eyes-msg:identifier-val is deprecated.  Use eyes-msg:identifier instead.")
  (identifier m))

(cl:ensure-generic-function 'left_forward-val :lambda-list '(m))
(cl:defmethod left_forward-val ((m <Generic>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader eyes-msg:left_forward-val is deprecated.  Use eyes-msg:left_forward instead.")
  (left_forward m))

(cl:ensure-generic-function 'right_forward-val :lambda-list '(m))
(cl:defmethod right_forward-val ((m <Generic>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader eyes-msg:right_forward-val is deprecated.  Use eyes-msg:right_forward instead.")
  (right_forward m))

(cl:ensure-generic-function 'left_speed-val :lambda-list '(m))
(cl:defmethod left_speed-val ((m <Generic>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader eyes-msg:left_speed-val is deprecated.  Use eyes-msg:left_speed instead.")
  (left_speed m))

(cl:ensure-generic-function 'right_speed-val :lambda-list '(m))
(cl:defmethod right_speed-val ((m <Generic>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader eyes-msg:right_speed-val is deprecated.  Use eyes-msg:right_speed instead.")
  (right_speed m))

(cl:ensure-generic-function 'timed-val :lambda-list '(m))
(cl:defmethod timed-val ((m <Generic>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader eyes-msg:timed-val is deprecated.  Use eyes-msg:timed instead.")
  (timed m))

(cl:ensure-generic-function 'duration-val :lambda-list '(m))
(cl:defmethod duration-val ((m <Generic>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader eyes-msg:duration-val is deprecated.  Use eyes-msg:duration instead.")
  (duration m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Generic>) ostream)
  "Serializes a message object of type '<Generic>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'left_forward) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'right_forward) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'left_speed)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'right_speed)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'timed) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'duration)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'duration)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'duration)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'duration)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Generic>) istream)
  "Deserializes a message object of type '<Generic>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'identifier)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'left_forward) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'right_forward) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'left_speed)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'right_speed)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'timed) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'duration)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'duration)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'duration)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'duration)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Generic>)))
  "Returns string type for a message object of type '<Generic>"
  "eyes/Generic")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Generic)))
  "Returns string type for a message object of type 'Generic"
  "eyes/Generic")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Generic>)))
  "Returns md5sum for a message object of type '<Generic>"
  "7a33c669a022f7fea29ccba33d517b1f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Generic)))
  "Returns md5sum for a message object of type 'Generic"
  "7a33c669a022f7fea29ccba33d517b1f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Generic>)))
  "Returns full string definition for message of type '<Generic>"
  (cl:format cl:nil "char identifier~%bool left_forward~%bool right_forward~%uint8 left_speed~%uint8 right_speed~%bool timed~%uint32 duration~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Generic)))
  "Returns full string definition for message of type 'Generic"
  (cl:format cl:nil "char identifier~%bool left_forward~%bool right_forward~%uint8 left_speed~%uint8 right_speed~%bool timed~%uint32 duration~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Generic>))
  (cl:+ 0
     1
     1
     1
     1
     1
     1
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Generic>))
  "Converts a ROS message object to a list"
  (cl:list 'Generic
    (cl:cons ':identifier (identifier msg))
    (cl:cons ':left_forward (left_forward msg))
    (cl:cons ':right_forward (right_forward msg))
    (cl:cons ':left_speed (left_speed msg))
    (cl:cons ':right_speed (right_speed msg))
    (cl:cons ':timed (timed msg))
    (cl:cons ':duration (duration msg))
))

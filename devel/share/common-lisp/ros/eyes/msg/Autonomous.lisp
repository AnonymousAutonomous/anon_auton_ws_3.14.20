; Auto-generated. Do not edit!


(cl:in-package eyes-msg)


;//! \htmlinclude Autonomous.msg.html

(cl:defclass <Autonomous> (roslisp-msg-protocol:ros-message)
  ((safety
    :reader safety
    :initarg :safety
    :type cl:boolean
    :initform cl:nil)
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
    :initform 0))
)

(cl:defclass Autonomous (<Autonomous>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Autonomous>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Autonomous)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name eyes-msg:<Autonomous> is deprecated: use eyes-msg:Autonomous instead.")))

(cl:ensure-generic-function 'safety-val :lambda-list '(m))
(cl:defmethod safety-val ((m <Autonomous>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader eyes-msg:safety-val is deprecated.  Use eyes-msg:safety instead.")
  (safety m))

(cl:ensure-generic-function 'left_forward-val :lambda-list '(m))
(cl:defmethod left_forward-val ((m <Autonomous>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader eyes-msg:left_forward-val is deprecated.  Use eyes-msg:left_forward instead.")
  (left_forward m))

(cl:ensure-generic-function 'right_forward-val :lambda-list '(m))
(cl:defmethod right_forward-val ((m <Autonomous>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader eyes-msg:right_forward-val is deprecated.  Use eyes-msg:right_forward instead.")
  (right_forward m))

(cl:ensure-generic-function 'left_speed-val :lambda-list '(m))
(cl:defmethod left_speed-val ((m <Autonomous>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader eyes-msg:left_speed-val is deprecated.  Use eyes-msg:left_speed instead.")
  (left_speed m))

(cl:ensure-generic-function 'right_speed-val :lambda-list '(m))
(cl:defmethod right_speed-val ((m <Autonomous>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader eyes-msg:right_speed-val is deprecated.  Use eyes-msg:right_speed instead.")
  (right_speed m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Autonomous>) ostream)
  "Serializes a message object of type '<Autonomous>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'safety) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'left_forward) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'right_forward) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'left_speed)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'right_speed)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Autonomous>) istream)
  "Deserializes a message object of type '<Autonomous>"
    (cl:setf (cl:slot-value msg 'safety) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'left_forward) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'right_forward) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'left_speed) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'right_speed) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Autonomous>)))
  "Returns string type for a message object of type '<Autonomous>"
  "eyes/Autonomous")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Autonomous)))
  "Returns string type for a message object of type 'Autonomous"
  "eyes/Autonomous")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Autonomous>)))
  "Returns md5sum for a message object of type '<Autonomous>"
  "4cbf5bb4d4e0610600c1d61a65d9f85d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Autonomous)))
  "Returns md5sum for a message object of type 'Autonomous"
  "4cbf5bb4d4e0610600c1d61a65d9f85d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Autonomous>)))
  "Returns full string definition for message of type '<Autonomous>"
  (cl:format cl:nil "bool safety~%bool left_forward~%bool right_forward~%int16 left_speed~%int16 right_speed~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Autonomous)))
  "Returns full string definition for message of type 'Autonomous"
  (cl:format cl:nil "bool safety~%bool left_forward~%bool right_forward~%int16 left_speed~%int16 right_speed~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Autonomous>))
  (cl:+ 0
     1
     1
     1
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Autonomous>))
  "Converts a ROS message object to a list"
  (cl:list 'Autonomous
    (cl:cons ':safety (safety msg))
    (cl:cons ':left_forward (left_forward msg))
    (cl:cons ':right_forward (right_forward msg))
    (cl:cons ':left_speed (left_speed msg))
    (cl:cons ':right_speed (right_speed msg))
))

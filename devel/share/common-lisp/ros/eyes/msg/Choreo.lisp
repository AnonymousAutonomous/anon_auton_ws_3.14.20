; Auto-generated. Do not edit!


(cl:in-package eyes-msg)


;//! \htmlinclude Choreo.msg.html

(cl:defclass <Choreo> (roslisp-msg-protocol:ros-message)
  ((timed
    :reader timed
    :initarg :timed
    :type cl:boolean
    :initform cl:nil)
   (duration
    :reader duration
    :initarg :duration
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
    :initform 0))
)

(cl:defclass Choreo (<Choreo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Choreo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Choreo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name eyes-msg:<Choreo> is deprecated: use eyes-msg:Choreo instead.")))

(cl:ensure-generic-function 'timed-val :lambda-list '(m))
(cl:defmethod timed-val ((m <Choreo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader eyes-msg:timed-val is deprecated.  Use eyes-msg:timed instead.")
  (timed m))

(cl:ensure-generic-function 'duration-val :lambda-list '(m))
(cl:defmethod duration-val ((m <Choreo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader eyes-msg:duration-val is deprecated.  Use eyes-msg:duration instead.")
  (duration m))

(cl:ensure-generic-function 'left_forward-val :lambda-list '(m))
(cl:defmethod left_forward-val ((m <Choreo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader eyes-msg:left_forward-val is deprecated.  Use eyes-msg:left_forward instead.")
  (left_forward m))

(cl:ensure-generic-function 'right_forward-val :lambda-list '(m))
(cl:defmethod right_forward-val ((m <Choreo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader eyes-msg:right_forward-val is deprecated.  Use eyes-msg:right_forward instead.")
  (right_forward m))

(cl:ensure-generic-function 'left_speed-val :lambda-list '(m))
(cl:defmethod left_speed-val ((m <Choreo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader eyes-msg:left_speed-val is deprecated.  Use eyes-msg:left_speed instead.")
  (left_speed m))

(cl:ensure-generic-function 'right_speed-val :lambda-list '(m))
(cl:defmethod right_speed-val ((m <Choreo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader eyes-msg:right_speed-val is deprecated.  Use eyes-msg:right_speed instead.")
  (right_speed m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Choreo>) ostream)
  "Serializes a message object of type '<Choreo>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'timed) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'duration)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Choreo>) istream)
  "Deserializes a message object of type '<Choreo>"
    (cl:setf (cl:slot-value msg 'timed) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'duration) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Choreo>)))
  "Returns string type for a message object of type '<Choreo>"
  "eyes/Choreo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Choreo)))
  "Returns string type for a message object of type 'Choreo"
  "eyes/Choreo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Choreo>)))
  "Returns md5sum for a message object of type '<Choreo>"
  "cc893b48a04f4c0dd26849bbdbb03ffa")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Choreo)))
  "Returns md5sum for a message object of type 'Choreo"
  "cc893b48a04f4c0dd26849bbdbb03ffa")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Choreo>)))
  "Returns full string definition for message of type '<Choreo>"
  (cl:format cl:nil "bool timed~%int32 duration~%bool left_forward~%bool right_forward~%int16 left_speed~%int16 right_speed~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Choreo)))
  "Returns full string definition for message of type 'Choreo"
  (cl:format cl:nil "bool timed~%int32 duration~%bool left_forward~%bool right_forward~%int16 left_speed~%int16 right_speed~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Choreo>))
  (cl:+ 0
     1
     4
     1
     1
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Choreo>))
  "Converts a ROS message object to a list"
  (cl:list 'Choreo
    (cl:cons ':timed (timed msg))
    (cl:cons ':duration (duration msg))
    (cl:cons ':left_forward (left_forward msg))
    (cl:cons ':right_forward (right_forward msg))
    (cl:cons ':left_speed (left_speed msg))
    (cl:cons ':right_speed (right_speed msg))
))
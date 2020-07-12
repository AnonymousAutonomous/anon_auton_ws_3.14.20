; Auto-generated. Do not edit!


(cl:in-package eyes-msg)


;//! \htmlinclude Big_Boi.msg.html

(cl:defclass <Big_Boi> (roslisp-msg-protocol:ros-message)
  ((timmy
    :reader timmy
    :initarg :timmy
    :type cl:boolean
    :initform cl:nil)
   (tommy
    :reader tommy
    :initarg :tommy
    :type cl:boolean
    :initform cl:nil)
   (tammy
    :reader tammy
    :initarg :tammy
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Big_Boi (<Big_Boi>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Big_Boi>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Big_Boi)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name eyes-msg:<Big_Boi> is deprecated: use eyes-msg:Big_Boi instead.")))

(cl:ensure-generic-function 'timmy-val :lambda-list '(m))
(cl:defmethod timmy-val ((m <Big_Boi>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader eyes-msg:timmy-val is deprecated.  Use eyes-msg:timmy instead.")
  (timmy m))

(cl:ensure-generic-function 'tommy-val :lambda-list '(m))
(cl:defmethod tommy-val ((m <Big_Boi>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader eyes-msg:tommy-val is deprecated.  Use eyes-msg:tommy instead.")
  (tommy m))

(cl:ensure-generic-function 'tammy-val :lambda-list '(m))
(cl:defmethod tammy-val ((m <Big_Boi>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader eyes-msg:tammy-val is deprecated.  Use eyes-msg:tammy instead.")
  (tammy m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Big_Boi>) ostream)
  "Serializes a message object of type '<Big_Boi>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'timmy) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'tommy) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'tammy) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Big_Boi>) istream)
  "Deserializes a message object of type '<Big_Boi>"
    (cl:setf (cl:slot-value msg 'timmy) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'tommy) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'tammy) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Big_Boi>)))
  "Returns string type for a message object of type '<Big_Boi>"
  "eyes/Big_Boi")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Big_Boi)))
  "Returns string type for a message object of type 'Big_Boi"
  "eyes/Big_Boi")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Big_Boi>)))
  "Returns md5sum for a message object of type '<Big_Boi>"
  "14735f0ab9bd89e9e2ae5c4a9db3ab4c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Big_Boi)))
  "Returns md5sum for a message object of type 'Big_Boi"
  "14735f0ab9bd89e9e2ae5c4a9db3ab4c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Big_Boi>)))
  "Returns full string definition for message of type '<Big_Boi>"
  (cl:format cl:nil "bool timmy~%bool tommy~%bool tammy~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Big_Boi)))
  "Returns full string definition for message of type 'Big_Boi"
  (cl:format cl:nil "bool timmy~%bool tommy~%bool tammy~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Big_Boi>))
  (cl:+ 0
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Big_Boi>))
  "Converts a ROS message object to a list"
  (cl:list 'Big_Boi
    (cl:cons ':timmy (timmy msg))
    (cl:cons ':tommy (tommy msg))
    (cl:cons ':tammy (tammy msg))
))

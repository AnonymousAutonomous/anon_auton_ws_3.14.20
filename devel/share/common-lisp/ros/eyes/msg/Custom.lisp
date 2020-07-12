; Auto-generated. Do not edit!


(cl:in-package eyes-msg)


;//! \htmlinclude Custom.msg.html

(cl:defclass <Custom> (roslisp-msg-protocol:ros-message)
  ((command
    :reader command
    :initarg :command
    :type cl:string
    :initform ""))
)

(cl:defclass Custom (<Custom>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Custom>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Custom)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name eyes-msg:<Custom> is deprecated: use eyes-msg:Custom instead.")))

(cl:ensure-generic-function 'command-val :lambda-list '(m))
(cl:defmethod command-val ((m <Custom>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader eyes-msg:command-val is deprecated.  Use eyes-msg:command instead.")
  (command m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Custom>) ostream)
  "Serializes a message object of type '<Custom>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'command))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'command))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Custom>) istream)
  "Deserializes a message object of type '<Custom>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'command) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'command) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Custom>)))
  "Returns string type for a message object of type '<Custom>"
  "eyes/Custom")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Custom)))
  "Returns string type for a message object of type 'Custom"
  "eyes/Custom")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Custom>)))
  "Returns md5sum for a message object of type '<Custom>"
  "cba5e21e920a3a2b7b375cb65b64cdea")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Custom)))
  "Returns md5sum for a message object of type 'Custom"
  "cba5e21e920a3a2b7b375cb65b64cdea")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Custom>)))
  "Returns full string definition for message of type '<Custom>"
  (cl:format cl:nil "string command~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Custom)))
  "Returns full string definition for message of type 'Custom"
  (cl:format cl:nil "string command~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Custom>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'command))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Custom>))
  "Converts a ROS message object to a list"
  (cl:list 'Custom
    (cl:cons ':command (command msg))
))

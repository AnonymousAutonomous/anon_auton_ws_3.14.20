;; Auto-generated. Do not edit!


(when (boundp 'eyes::Generic)
  (if (not (find-package "EYES"))
    (make-package "EYES"))
  (shadow 'Generic (find-package "EYES")))
(unless (find-package "EYES::GENERIC")
  (make-package "EYES::GENERIC"))

(in-package "ROS")
;;//! \htmlinclude Generic.msg.html


(defclass eyes::Generic
  :super ros::object
  :slots (_identifier _left_forward _right_forward _left_speed _right_speed _timed _duration ))

(defmethod eyes::Generic
  (:init
   (&key
    ((:identifier __identifier) 0)
    ((:left_forward __left_forward) nil)
    ((:right_forward __right_forward) nil)
    ((:left_speed __left_speed) 0)
    ((:right_speed __right_speed) 0)
    ((:timed __timed) nil)
    ((:duration __duration) 0)
    )
   (send-super :init)
   (setq _identifier (round __identifier))
   (setq _left_forward __left_forward)
   (setq _right_forward __right_forward)
   (setq _left_speed (round __left_speed))
   (setq _right_speed (round __right_speed))
   (setq _timed __timed)
   (setq _duration (round __duration))
   self)
  (:identifier
   (&optional __identifier)
   (if __identifier (setq _identifier __identifier)) _identifier)
  (:left_forward
   (&optional __left_forward)
   (if __left_forward (setq _left_forward __left_forward)) _left_forward)
  (:right_forward
   (&optional __right_forward)
   (if __right_forward (setq _right_forward __right_forward)) _right_forward)
  (:left_speed
   (&optional __left_speed)
   (if __left_speed (setq _left_speed __left_speed)) _left_speed)
  (:right_speed
   (&optional __right_speed)
   (if __right_speed (setq _right_speed __right_speed)) _right_speed)
  (:timed
   (&optional __timed)
   (if __timed (setq _timed __timed)) _timed)
  (:duration
   (&optional __duration)
   (if __duration (setq _duration __duration)) _duration)
  (:serialization-length
   ()
   (+
    ;; char _identifier
    1
    ;; bool _left_forward
    1
    ;; bool _right_forward
    1
    ;; uint8 _left_speed
    1
    ;; uint8 _right_speed
    1
    ;; bool _timed
    1
    ;; uint32 _duration
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; char _identifier
       (write-byte _identifier s)
     ;; bool _left_forward
       (if _left_forward (write-byte -1 s) (write-byte 0 s))
     ;; bool _right_forward
       (if _right_forward (write-byte -1 s) (write-byte 0 s))
     ;; uint8 _left_speed
       (write-byte _left_speed s)
     ;; uint8 _right_speed
       (write-byte _right_speed s)
     ;; bool _timed
       (if _timed (write-byte -1 s) (write-byte 0 s))
     ;; uint32 _duration
       (write-long _duration s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; char _identifier
     (setq _identifier (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; bool _left_forward
     (setq _left_forward (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _right_forward
     (setq _right_forward (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; uint8 _left_speed
     (setq _left_speed (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _right_speed
     (setq _right_speed (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; bool _timed
     (setq _timed (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; uint32 _duration
     (setq _duration (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get eyes::Generic :md5sum-) "7a33c669a022f7fea29ccba33d517b1f")
(setf (get eyes::Generic :datatype-) "eyes/Generic")
(setf (get eyes::Generic :definition-)
      "char identifier
bool left_forward
bool right_forward
uint8 left_speed
uint8 right_speed
bool timed
uint32 duration

")



(provide :eyes/Generic "7a33c669a022f7fea29ccba33d517b1f")



;; Auto-generated. Do not edit!


(when (boundp 'eyes::Choreo)
  (if (not (find-package "EYES"))
    (make-package "EYES"))
  (shadow 'Choreo (find-package "EYES")))
(unless (find-package "EYES::CHOREO")
  (make-package "EYES::CHOREO"))

(in-package "ROS")
;;//! \htmlinclude Choreo.msg.html


(defclass eyes::Choreo
  :super ros::object
  :slots (_timed _duration _left_forward _right_forward _left_speed _right_speed ))

(defmethod eyes::Choreo
  (:init
   (&key
    ((:timed __timed) nil)
    ((:duration __duration) 0)
    ((:left_forward __left_forward) nil)
    ((:right_forward __right_forward) nil)
    ((:left_speed __left_speed) 0)
    ((:right_speed __right_speed) 0)
    )
   (send-super :init)
   (setq _timed __timed)
   (setq _duration (round __duration))
   (setq _left_forward __left_forward)
   (setq _right_forward __right_forward)
   (setq _left_speed (round __left_speed))
   (setq _right_speed (round __right_speed))
   self)
  (:timed
   (&optional __timed)
   (if __timed (setq _timed __timed)) _timed)
  (:duration
   (&optional __duration)
   (if __duration (setq _duration __duration)) _duration)
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
  (:serialization-length
   ()
   (+
    ;; bool _timed
    1
    ;; int32 _duration
    4
    ;; bool _left_forward
    1
    ;; bool _right_forward
    1
    ;; int16 _left_speed
    2
    ;; int16 _right_speed
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _timed
       (if _timed (write-byte -1 s) (write-byte 0 s))
     ;; int32 _duration
       (write-long _duration s)
     ;; bool _left_forward
       (if _left_forward (write-byte -1 s) (write-byte 0 s))
     ;; bool _right_forward
       (if _right_forward (write-byte -1 s) (write-byte 0 s))
     ;; int16 _left_speed
       (write-word _left_speed s)
     ;; int16 _right_speed
       (write-word _right_speed s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _timed
     (setq _timed (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; int32 _duration
     (setq _duration (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool _left_forward
     (setq _left_forward (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _right_forward
     (setq _right_forward (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; int16 _left_speed
     (setq _left_speed (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _right_speed
     (setq _right_speed (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get eyes::Choreo :md5sum-) "cc893b48a04f4c0dd26849bbdbb03ffa")
(setf (get eyes::Choreo :datatype-) "eyes/Choreo")
(setf (get eyes::Choreo :definition-)
      "bool timed
int32 duration
bool left_forward
bool right_forward
int16 left_speed
int16 right_speed

")



(provide :eyes/Choreo "cc893b48a04f4c0dd26849bbdbb03ffa")



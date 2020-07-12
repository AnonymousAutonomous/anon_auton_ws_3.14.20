;; Auto-generated. Do not edit!


(when (boundp 'eyes::Autonomous)
  (if (not (find-package "EYES"))
    (make-package "EYES"))
  (shadow 'Autonomous (find-package "EYES")))
(unless (find-package "EYES::AUTONOMOUS")
  (make-package "EYES::AUTONOMOUS"))

(in-package "ROS")
;;//! \htmlinclude Autonomous.msg.html


(defclass eyes::Autonomous
  :super ros::object
  :slots (_safety _left_forward _right_forward _left_speed _right_speed ))

(defmethod eyes::Autonomous
  (:init
   (&key
    ((:safety __safety) nil)
    ((:left_forward __left_forward) nil)
    ((:right_forward __right_forward) nil)
    ((:left_speed __left_speed) 0)
    ((:right_speed __right_speed) 0)
    )
   (send-super :init)
   (setq _safety __safety)
   (setq _left_forward __left_forward)
   (setq _right_forward __right_forward)
   (setq _left_speed (round __left_speed))
   (setq _right_speed (round __right_speed))
   self)
  (:safety
   (&optional __safety)
   (if __safety (setq _safety __safety)) _safety)
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
    ;; bool _safety
    1
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
     ;; bool _safety
       (if _safety (write-byte -1 s) (write-byte 0 s))
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
   ;; bool _safety
     (setq _safety (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
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

(setf (get eyes::Autonomous :md5sum-) "4cbf5bb4d4e0610600c1d61a65d9f85d")
(setf (get eyes::Autonomous :datatype-) "eyes/Autonomous")
(setf (get eyes::Autonomous :definition-)
      "bool safety
bool left_forward
bool right_forward
int16 left_speed
int16 right_speed

")



(provide :eyes/Autonomous "4cbf5bb4d4e0610600c1d61a65d9f85d")



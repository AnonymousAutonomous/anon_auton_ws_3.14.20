;; Auto-generated. Do not edit!


(when (boundp 'eyes::Big_Boi)
  (if (not (find-package "EYES"))
    (make-package "EYES"))
  (shadow 'Big_Boi (find-package "EYES")))
(unless (find-package "EYES::BIG_BOI")
  (make-package "EYES::BIG_BOI"))

(in-package "ROS")
;;//! \htmlinclude Big_Boi.msg.html


(defclass eyes::Big_Boi
  :super ros::object
  :slots (_timmy _tommy _tammy ))

(defmethod eyes::Big_Boi
  (:init
   (&key
    ((:timmy __timmy) nil)
    ((:tommy __tommy) nil)
    ((:tammy __tammy) nil)
    )
   (send-super :init)
   (setq _timmy __timmy)
   (setq _tommy __tommy)
   (setq _tammy __tammy)
   self)
  (:timmy
   (&optional __timmy)
   (if __timmy (setq _timmy __timmy)) _timmy)
  (:tommy
   (&optional __tommy)
   (if __tommy (setq _tommy __tommy)) _tommy)
  (:tammy
   (&optional __tammy)
   (if __tammy (setq _tammy __tammy)) _tammy)
  (:serialization-length
   ()
   (+
    ;; bool _timmy
    1
    ;; bool _tommy
    1
    ;; bool _tammy
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _timmy
       (if _timmy (write-byte -1 s) (write-byte 0 s))
     ;; bool _tommy
       (if _tommy (write-byte -1 s) (write-byte 0 s))
     ;; bool _tammy
       (if _tammy (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _timmy
     (setq _timmy (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _tommy
     (setq _tommy (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _tammy
     (setq _tammy (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get eyes::Big_Boi :md5sum-) "14735f0ab9bd89e9e2ae5c4a9db3ab4c")
(setf (get eyes::Big_Boi :datatype-) "eyes/Big_Boi")
(setf (get eyes::Big_Boi :definition-)
      "bool timmy
bool tommy
bool tammy

")



(provide :eyes/Big_Boi "14735f0ab9bd89e9e2ae5c4a9db3ab4c")



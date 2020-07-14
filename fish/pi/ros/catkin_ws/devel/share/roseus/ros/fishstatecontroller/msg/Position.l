;; Auto-generated. Do not edit!


(when (boundp 'fishstatecontroller::Position)
  (if (not (find-package "FISHSTATECONTROLLER"))
    (make-package "FISHSTATECONTROLLER"))
  (shadow 'Position (find-package "FISHSTATECONTROLLER")))
(unless (find-package "FISHSTATECONTROLLER::POSITION")
  (make-package "FISHSTATECONTROLLER::POSITION"))

(in-package "ROS")
;;//! \htmlinclude Position.msg.html


(defclass fishstatecontroller::Position
  :super ros::object
  :slots (_distance _x_offset _y_offset ))

(defmethod fishstatecontroller::Position
  (:init
   (&key
    ((:distance __distance) "")
    ((:x_offset __x_offset) "")
    ((:y_offset __y_offset) "")
    )
   (send-super :init)
   (setq _distance (string __distance))
   (setq _x_offset (string __x_offset))
   (setq _y_offset (string __y_offset))
   self)
  (:distance
   (&optional __distance)
   (if __distance (setq _distance __distance)) _distance)
  (:x_offset
   (&optional __x_offset)
   (if __x_offset (setq _x_offset __x_offset)) _x_offset)
  (:y_offset
   (&optional __y_offset)
   (if __y_offset (setq _y_offset __y_offset)) _y_offset)
  (:serialization-length
   ()
   (+
    ;; string _distance
    4 (length _distance)
    ;; string _x_offset
    4 (length _x_offset)
    ;; string _y_offset
    4 (length _y_offset)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _distance
       (write-long (length _distance) s) (princ _distance s)
     ;; string _x_offset
       (write-long (length _x_offset) s) (princ _x_offset s)
     ;; string _y_offset
       (write-long (length _y_offset) s) (princ _y_offset s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _distance
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _distance (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _x_offset
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _x_offset (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _y_offset
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _y_offset (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get fishstatecontroller::Position :md5sum-) "7269bbef7b024d4726181ff666ab9ee9")
(setf (get fishstatecontroller::Position :datatype-) "fishstatecontroller/Position")
(setf (get fishstatecontroller::Position :definition-)
      "string distance
string x_offset
string y_offset

")



(provide :fishstatecontroller/Position "7269bbef7b024d4726181ff666ab9ee9")



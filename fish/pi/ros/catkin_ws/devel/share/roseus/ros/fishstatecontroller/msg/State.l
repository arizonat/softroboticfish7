;; Auto-generated. Do not edit!


(when (boundp 'fishstatecontroller::State)
  (if (not (find-package "FISHSTATECONTROLLER"))
    (make-package "FISHSTATECONTROLLER"))
  (shadow 'State (find-package "FISHSTATECONTROLLER")))
(unless (find-package "FISHSTATECONTROLLER::STATE")
  (make-package "FISHSTATECONTROLLER::STATE"))

(in-package "ROS")
;;//! \htmlinclude State.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass fishstatecontroller::State
  :super ros::object
  :slots (_header _state _adjust ))

(defmethod fishstatecontroller::State
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:state __state) "")
    ((:adjust __adjust) "")
    )
   (send-super :init)
   (setq _header __header)
   (setq _state (string __state))
   (setq _adjust (string __adjust))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:state
   (&optional __state)
   (if __state (setq _state __state)) _state)
  (:adjust
   (&optional __adjust)
   (if __adjust (setq _adjust __adjust)) _adjust)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _state
    4 (length _state)
    ;; string _adjust
    4 (length _adjust)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _state
       (write-long (length _state) s) (princ _state s)
     ;; string _adjust
       (write-long (length _adjust) s) (princ _adjust s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _state
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _state (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _adjust
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _adjust (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get fishstatecontroller::State :md5sum-) "f3791bcc7f7d0d52403bf5e36635c950")
(setf (get fishstatecontroller::State :datatype-) "fishstatecontroller/State")
(setf (get fishstatecontroller::State :definition-)
      "Header header
#time clock
string state
string adjust

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

")



(provide :fishstatecontroller/State "f3791bcc7f7d0d52403bf5e36635c950")



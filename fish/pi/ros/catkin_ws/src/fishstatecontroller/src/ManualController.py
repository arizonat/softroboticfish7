#!/usr/bin/env python

import cv2
import rospy
import roslib
import numpy as np
from std_msgs.msg import String, Float64

class ManualController():
    """
    Control SoFi by pressing buttons on a keyboard
    Key mappings:
        - 'a': hard left
        - 's': soft left
        - 'd': go forward
        - 'f': soft right
        - 'g': full right
        - (else): do nothing
    """
    def __init__(self):
        self.heading_pub = rospy.Publisher('heading_cmd', Float64, queue_size=10)
        self.pitch_pub = rospy.Publisher('pitch_cmd', Float64, queue_size=10)
        self.thrust_pub = rospy.Publisher('thrust_cmd', Float64, queue_size=10)

        #Values between -1 and 1 inclusive
        self.heading = 0.0      #-1 for full left, +1 for full right
        self.pitch = 0.0        
        self.thrust = 0.34

        self.image = np.zeros((350,450,3), np.uint8)
        instructions = ["CLICK HERE", "Press and hold the", "following buttons to move SoFi","", "a: HARD LEFT", "s: SOFT LEFT", "d: GO FORWARD", "f: SOFT RIGHT", "g: HARD RIGHT", "else: DO NOTHING"]
        org = (20, 25)
        font = cv2.FONT_HERSHEY_SIMPLEX
        color = (255, 255, 255)
        for line in instructions:
            cv2.putText(self.image, line, org, font, 1, color, 2)
            org = (org[0], org[1] + 35)
    
    def run(self):
        rate = rospy.Rate(24)
        while not rospy.is_shutdown():
            #read for keyboard presses and set appropriate values for heading, pitch, and thrust
            cv2.imshow("blank", self.image)            
            key = cv2.waitKey(0)
            if key == ord('a'):         #HARD LEFT
                self.heading = -1.0
                self.thrust = -0.33
            elif key == ord('s'):       #SOFT LEFT
                self.heading = -0.72
                self.thrust = -0.33
            elif key == ord('d'):       #GO FORWARD
                self.heading = 0.0
                self.thrust = -1.0
            elif key == ord('f'):       #SOFT RIGHT
                self.heading = 0.72
                self.thrust = -0.33
            elif key == ord('g'):       #HARD RIGHT
                self.heading = 1.0
                self.thrust = -0.33
            else:                       #DO NOTHING
                self.heading = 0.0
                self.thrust = 0.34

            #publish the appropriate commands to the *_cmd topics
            self.heading_pub.publish(self.heading)
            self.pitch_pub.publish(self.pitch)
            self.thrust_pub.publish(self.thrust)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('manual_controller', anonymous=True)  
    controller = ManualController()
    print("\nStarting Manual Controller at 24hz")
    controller.run()
    print("\nManual Controller: done")

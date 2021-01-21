#!/usr/bin/env python

import cv2
import rospy
import roslib
import numpy as np
from std_msgs.msg import String, Float64

class Visualization():
    """Create markers to be used in an RVIZ visualization"""
    def __init__(self):
        pass

    def run(self):
        #Create marker for fish position at (0,0,0)
        #Create marker for estimated target position
        #Create marker for target average position
        #Pose for heading_cmd effort
        #Pose for pitch_cmd effort
        #Pose for thrust_cmd effort
        pass

    def position_callback(self, rosdata):
        pass

    def average_position_callback(self, rosdata):
        pass

    def heading_cmd_callback(self, rosdata):
        pass

    def pitch_cmd_callback(self, rosdata):
        pass

    def thrust_cmd_callback(self, rosdata):;
        pass

if __name__ == '__main__':
    rospy.init_node('visualization', anonymous=True)  
    viz = Visualization()
    print("\nVisualization: Beginning at 24hz")
    viz.run()
    print("\nVisualization: done")

#!/usr/bin/env python

# FishStateMachine:
# Implementation of the finite state machine for SoFi (soft robotic fish)
#
# Node name: finite_state_machine
# Subscribed topics:
#   - fish_pose
#   - target_found
#   - average_heading
#   - average_pitch (TODO)
#   - target_centroid (TODO)
# Published topics:
#   - pid_enable
#   - heading_state
#   - heading_setpoint
#   - pitch_state (TODO)
#   - pitch_setpoint (TODO)
# Manual Testing Notes (TODO)
# 

import rospy
import roslib
#import serial # see http://pyserial.readthedocs.org/en/latest/pyserial_api.html
from time import time, sleep
import time as clock
from std_msgs.msg import String, Float64, Bool
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
import cv2
from cv_bridge import CvBridge
import numpy as np
from fishstatecontroller.msg import State, Position

class FishStateController():
    def __init__(self, update_hz, heading_setpoint=0.0):
        """
        update_hz: the update rate of the state machine
        """
        ###State information, message, and publisher###
        self.state = None
        self.states = ("INIT","SEARCH","FOLLOW")
        self.state_msg = State()
        self.state_pub = rospy.Publisher('fish_state', State, queue_size=10)
        #self.state_pub = rospy.Publisher('fish_state', String, queue_size=10)

        self.update_hz = update_hz
        self.pid_enable_pub = rospy.Publisher('pid_enable', Bool, queue_size=10)
        self.target_found = False
        self.fish_pose = PoseStamped()

        ###heading, pitch, and distance state and setpoint information###
        self.heading_state = None
        self.heading_state_pub = rospy.Publisher('heading_state', Float64, queue_size=10)
        self.heading_setpoint = heading_setpoint
        self.heading_setpoint_pub = rospy.Publisher('heading_setpoint', Float64, queue_size=10)
        # TODO pitch state and setpoint parameters and publishers
        # TODO distance state and setpoint parameters and publishers
        self.transitionTo("INIT")

    def run(self):
        rate = rospy.Rate(self.update_hz)
        while not rospy.is_shutdown():
            if self.state == "INIT":
                self.pid_enable_pub.publish(False)
                sleep(15)
                self.transitionTo("SEARCH")
            
            elif self.state == "SEARCH":
                if self.target_found:
                    self.transitionTo("FOLLOW")
                    self.update()
                else:
                    self.pid_enable_pub.publish(False)

            elif self.state == "FOLLOW":
                if self.target_found:
                    self.update()
                    print("Following target at: %d"%(self.fish_pose.pose.position))
                else:
                    self.transitionTo("SEARCH")
                    self.pid_enable_pub.publish(False)
            rate.sleep()

    def transitionTo(self, state_name):
        self.state = state_name
        print(self.state)
        ###Can use below if adjust direction not important.
        self.state_msg.header.stamp = rospy.Time.now()
        self.state_msg.state = self.state
        self.state_pub.publish(self.state_msg)

    def update(self):
        self.pid_enable_pub.publish(True)
        self.heading_state_pub.publish(self.heading_state)
        self.heading_setpoint_pub.publish(self.heading_setpoint)

    def heading_callback(self, ros_data):
        self.heading_state = ros_data

    def found_callback(self, ros_data):
        self.target_found = ros_data

    def pose_callback(self, ros_data):
        self.fish_pose = ros_data

if __name__ == '__main__':
    rospy.init_node('finite_state_machine', anonymous=True)
    update_hz = 24
    state_machine = FishStateController(update_hz)
 
    rospy.Subscriber('average_heading', Float64, state_machine.heading_callback)
    rospy.Subscriber('target_found', Bool, state_machine.found_callback)
    rospy.Subscriber('fish_pose', PoseStamped, state_machine.pose_callback)

    print("Beginning finite state machine at %d hz\n"%(update_hz))
    state_machine.run()
    print("Done\n")
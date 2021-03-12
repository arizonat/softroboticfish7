#!/usr/bin/env python

# FishStateMachine:
# Implementation of the finite state machine for SoFi (soft robotic fish)
#
# Node name: finite_state_machine
# Subscribed topics:
#   - fish_pose
#   - target_found
#   - average_heading
#   - average_pitch
#   - average_dist
#   - target_centroid (TODO)
# Published topics:
#   - pid_enable
#   - heading_state
#   - heading_setpoint
#   - pitch_state
#   - pitch_setpoint 
#   - dist_state
#   - dist_setpoint
#   - heading_cmd
#   - pitch_cmd
#   - thrust_cmd

# Manual Testing Notes
# The results of each published topic in this node was tested for the following inputs:
#   - target_found (topic) was True
#   - target_found (topic) was False
# Both of the above inputs were tested in each state (INIT, SEARCH, FOLLOW) of the state machine.
# The program changed states and published to the pid_enable and heading... topics correctly 

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
    def __init__(self, update_hz, heading_setpoint=0.0, pitch_setpoint=0.0, dist_setpoint = 3.0):
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

        self.pitch_state = None
        self.pitch_state_pub = rospy.Publisher('pitch_state', Float64, queue_size=10)
        self.pitch_setpoint = pitch_setpoint
        self.pitch_setpoint_pub = rospy.Publisher('pitch_setpoint', Float64, queue_size=10)

        self.dist_state = None
        self.dist_state_pub = rospy.Publisher('dist_state', Float64, queue_size=10)
        self.dist_setpoint = dist_setpoint
        self.dist_setpoint_pub = rospy.Publisher('dist_setpoint', Float64, queue_size=10)

        self.heading_cmd_pub = rospy.Publisher('heading_cmd', Float64, queue_size=10)
        self.pitch_cmd_pub = rospy.Publisher('pitch_cmd', Float64, queue_size=10)
        self.thrust_cmd_pub = rospy.Publisher('thrust_cmd', Float64, queue_size=10)

        self.search_direction = None   #1 for search RIGHT, -1 for search LEFT

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
                    self.publish_states()
                else:
                    self.pid_enable_pub.publish(False)
                    self.publish_search_cmd() #Publish HARD LEFT

            elif self.state == "FOLLOW":
                if self.target_found:
                    self.publish_states()
                    #print("Following target at: %f, %f"%(self.fish_pose.pose.position.y, self.fish_pose.pose.position.z))
                    direction = self.fish_pose.pose.position.y
                    self.search_direction = - direction / abs(direction) #scale to +1/-1
                else:
                    self.transitionTo("SEARCH")
                    self.pid_enable_pub.publish(False)
            rate.sleep()

    def transitionTo(self, state_name):
        self.state = state_name
        #print(self.state)
        ###Can use below if adjust direction not important.
        self.state_msg.header.stamp = rospy.Time.now()
        self.state_msg.state = self.state
        self.state_pub.publish(self.state_msg)

    def publish_states(self):
        self.pid_enable_pub.publish(True)
        #heading
        self.heading_state_pub.publish(self.heading_state)
        self.heading_setpoint_pub.publish(self.heading_setpoint)
        #pitch
        self.pitch_state_pub.publish(self.pitch_state)
        self.pitch_setpoint_pub.publish(self.pitch_setpoint)
        #distance
        self.dist_state_pub.publish(self.dist_state)
        self.dist_setpoint_pub.publish(self.dist_setpoint)

    def publish_search_cmd(self):
        #Publish a hard left to the heading, pitch, and thrust commands if in SEARCH state
        if self.search_direction is not None:
            self.heading_cmd_pub.publish(self.search_direction)
            self.pitch_cmd_pub.publish(0)
            self.thrust_cmd_pub.publish(-1)

    def heading_callback(self, ros_data):
        self.heading_state = ros_data

    def pitch_callback(self, ros_data):
        self.pitch_state = ros_data

    def dist_callback(self, ros_data):
        self.dist_state = ros_data

    def found_callback(self, ros_data):
        self.target_found = ros_data.data

    def pose_callback(self, ros_data):
        self.fish_pose = ros_data

if __name__ == '__main__':
    rospy.init_node('finite_state_machine', anonymous=True)
    update_hz = 24
    heading_setpoint = rospy.get_param("~heading_setpoint", 0.0)
    
    state_machine = FishStateController(update_hz, heading_setpoint=heading_setpoint)
 
    rospy.Subscriber('average_heading', Float64, state_machine.heading_callback)
    rospy.Subscriber('average_pitch', Float64, state_machine.pitch_callback)
    rospy.Subscriber('average_dist', Float64, state_machine.dist_callback)

    rospy.Subscriber('target_found', Bool, state_machine.found_callback)
    rospy.Subscriber('fish_pose', PoseStamped, state_machine.pose_callback)

    print("Fish State Machine: Beginning at %d hz\n"%(update_hz))
    state_machine.run()
    print("\nFish State Machine: done\n")

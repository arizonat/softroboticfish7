#!/usr/bin/env python

import cv2
import rospy
import roslib
import numpy as np
from std_msgs.msg import String, Float64
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
from math import tan

class Visualization():
    """Create markers to be used in an RVIZ visualization"""
    def __init__(self):
        self.target = Marker()
        self.target_pub = rospy.Publisher('target_marker', Marker, queue_size=10)
        self.target_avg = Marker()
        self.target_avg_pub = rospy.Publisher('target_avg_marker', Marker, queue_size=10)

        self.heading = 0.0
        self.heading_avg = 0.0
        self.pitch = 0.0
        self.dist = 0.0

    def run(self):
        rate = rospy.Rate(24)
        while not rospy.is_shutdown():        
            #Create marker for estimated target position
            x = self.dist
            y = self.dist * tan(self.heading) * -1.0
            z = self.dist * tan(self.pitch)
            self.set_marker_attributes(self.target, 1, x, y, z)

            #Create marker for target average position
            y_avg = self.dist * tan(self.heading_avg) * -1.0
            self.set_marker_attributes(self.target_avg, 2, x, y_avg, z, r=0, g=1, b=0, scale=0.5)

            #Publish the Markers
            self.target_pub.publish(self.target)
            self.target_avg_pub.publish(self.target_avg)
    
    def set_marker_attributes(self, marker, seq, x, y, z, r=1, g=0, b=0, scale=1, lifetime=1):
        #Header and Action
        marker.header.seq = seq
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = "sofi_cam"
        marker.action = 0 #ADD

        #Object shape and pose
        marker.type =  2 #SPHERE
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1
        
        #Object color and scale
        marker.color.a = 0.75
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale

        marker.lifetime = rospy.Duration.from_sec(lifetime)

    def position_callback(self, rosdata):
        self.heading = rosdata.pose.position.y
        self.pitch = rosdata.pose.position.z
        self.dist = rosdata.pose.position.x

    def average_position_callback(self, rosdata):
        self.heading_avg = rosdata.data

if __name__ == '__main__':
    rospy.init_node('visualization', anonymous=True)  
    viz = Visualization()
    rospy.Subscriber('fish_pose', PoseStamped, viz.position_callback)
    rospy.Subscriber('average_heading', Float64, viz.average_position_callback)
    print("\nVisualization: Beginning at 24hz")
    viz.run()
    print("\nVisualization: done")

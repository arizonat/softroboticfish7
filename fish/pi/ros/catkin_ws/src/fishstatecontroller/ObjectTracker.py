#!/usr/bin/env python

# ObjectTracker:
# Implementation of state estimation for SoFi (soft robotic fish)
#
# Node name: state_estimation
# Subscribed topics:
#   - /raspicam_node/image/compressed
# Published topics:
#   - fish_pose
#   - color_mask
#   - target_found
#   - average_heading
#   - average_pitch (TODO)

import rospy
import roslib
from std_msgs.msg import String, Float64, Bool
from sensor_msgs.msg import Image, CompressedImage
for geometry_msgs.msg import PoseStamped
import cv2
from cv_bridge import CvBridge
import numpy as np
from fishstatecontroller.msg import State, Position

class ObjectTracker():
    def __init__(self):
        self.image = np.zeros((960,1280,3), np.uint8)
        self.subsample_ratio = 0.25             # amount to shrink image, maintains aspect ratio
        self.focal_length = 993.0               #the focal length of the camera
        self.real_height = 1.25                 #the real height of the target object
        self.image_center = (635.08, 469.80)    #the image center of the camera

        self.pose = PoseStamped()
        self.pose_pub = rospy.Publisher('fish_pose', PoseStamped, queue_size=10)
        self.mask_pub = rospy.Publisher('color_mask', Image, queue_size=10)
        self.found_pub = rospy.Publisher('target_found', Bool, queue_size=10)
        self.average_heading_pub = rospy.Publisher('average_heading', Float64, queue_size=10)
        #self.average_pitch_pub = TODO

        # Initiate position msg instance and new publisher for data
        #self.position_msg = Position()
        #self.position_pub = rospy.Publisher('fish_position', Position, queue_size=10)

        # red stretches 2 bands in hsv
        # these values are for yellow, keeping the 2 bands for red in the future
        self.hsv_lower_lower = (14,55,55)
        self.hsv_lower_upper = (30,255,235)
        self.hsv_upper_lower = self.hsv_lower_lower
        self.hsv_upper_upper = self.hsv_lower_upper

        #for heading averaging
        self.current_sum = 0
        self.count = 0
        self.current_slope = 0
        self.last_point = 0
        self.current_point = 0
        self.average = 0

    def find_target(self):
        target_found, target_centroid, dist, offset = self.process_image(self.image)
        return (target_found, target_centroid, dist, offset)

    def process_image(self, image):
        """
        Processes a single image, looks for a circle, return the target centroid in the camera frame
        Reference: https://www.pyimagesearch.com/2015/09/14/ball-tracking-with-opencv/
        """
        target_found = False
        target_centroid = None

        #Blur and resize the image, put into HSV color scale, and create an image mask
        img_small = cv2.resize(image, None, fx=self.subsample_ratio, fy=self.subsample_ratio, interpolation=cv2.INTER_LINEAR)
        img_small = cv2.GaussianBlur(img_small, (5,5), 0)
        hsv_small = cv2.cvtColor(img_small, cv2.COLOR_BGR2HSV)
        mask_l = cv2.inRange(hsv_small, self.hsv_lower_lower, self.hsv_lower_upper)
        mask_u = cv2.inRange(hsv_small, self.hsv_upper_lower, self.hsv_upper_upper)
        mask = cv2.bitwise_or(mask_l, mask_u)

        #Publish the mask
        mask_bgr8 = cv2.cvtColor(mask,cv2.COLOR_GRAY2BGR)
        bridge = CvBridge()
        cv_mask = bridge.cv2_to_imgmsg(mask_bgr8, encoding='bgr8')
        self.mask_pub.publish(cv_mask)

        #find the largest contour of the mask or return False,None if target is not there
        cnts, cnt_hier = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2:]
        if len(cnts) == 0:
          return (False, None, None, None)
        cnt = max(cnts, key=cv2.contourArea)
        ((x,y),radius) = cv2.minEnclosingCircle(cnt)

        if radius < 5:
          return (False, None, None, None)

        target_centroid = ((int(x/self.subsample_ratio),int(y/self.subsample_ratio)),int(radius/self.subsample_ratio))
        target_found = True

        #Calculate the object's distance and offset from center
        height_px =  2 * target_centroid[1]
        offset_px = -1.0*(target_centroid[0][0] - self.image_center[0]) , -1.0*(target_centroid[0][1] - self.image_center[1])
        distance = (self.focal_length * self.real_height) / height_px
        y_offset = (offset_px[0] * distance)/self.focal_length
        z_offset = (offset_px[1] * distance)/self.focal_length

        # Publish distance and offset information to message file instead
        #self.position_msg.distance = str(distance)
        #self.position_msg.x_offset = str(z_offset)
        #self.position_msg.y_offset = str(y_offset)
        #self.position_pub.publish(self.position_msg)

        return (target_found, target_centroid, distance, (y_offset, z_offset))

    def run(self):
        """Run the Distance Tracker with the input from the camera"""
        bridge = CvBridge()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            target_found, target_centroid, dist, offset = self.find_target()
            #print("EST DISTANCE: " + str(dist) + ' inches')
            #print("EST OFFSET: " + str(offset) + ' inches')
            #print("--------------")
            if target_found:
                #heading averaging
                self.last_point = self.current_point
                self.current_point = offset[0]
                new_slope = (self.current_point - self.last_point)
                if new_slope != 0:
                    new_slope = new_slope / abs(new_slope)
                if self.current_slope == 1:
                    if new_slope < 0:
                        self.average = self.current_sum / self.count
                        self.current_sum = 0
                        self.count = 0
                self.current_sum += self.current_point
                self.count += 1
                self.current_slope = new_slope
                #TODO pitch averaging

                #Publish everything
                self.pose.header.seq = 1
                self.pose.header.stamp = rospy.Time.now()
                self.pose.header.frame_id = "sofi_cam"
                self.pose.pose.position.x = distance
                self.pose.pose.position.y = offset[0]
                self.pose.pose.position.z = offset[1]
                self.pose.pose.orientation.x = 0
                self.pose.pose.orientation.y = 0
                self.pose.pose.orientation.z = 0
                self.pose.pose.orientation.w = 1

                self.pose_pub.publish(self.pose)
                self.found_pub.publish(True)
                self.average_heading_pub.publish(self.average)
                #TODO publish the pitch average

            else:
                self.found_pub.publish(False)

            rate.sleep()

    def callback(self, ros_data):
        bridge = CvBridge()
        self.image = bridge.compressed_imgmsg_to_cv2(ros_data, desired_encoding='passthrough')

if __name__ == '__main__':
    rospy.init_node('state_estimation', anonymous=True)
    tracker = ObjectTracker()
    rospy.Subscriber('/raspicam_node/image/compressed', CompressedImage, tracker.callback)
    print("Beginning position tracker at 10hz\n")
    tracker.run()
    print("Done\n")

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
#   - average_pitch
#   - average_dist
# Manual Testing Notes
# The result of each published topic was analyzed for the following inputs:
#   - No input camera image (raspicam_node not running)
#   - Target not found in picture
#   - Stationary target found in picture
#   - Moving target found in picture
# All published topics behaved as expected
#
# TODO Perform some kind of averaging on the pitch and distance measurements (i.e.
# a moving average with a 5-10 sample window size); see other TODOs

import rospy
import roslib
from math import atan2
from std_msgs.msg import String, Float64, Bool
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import PoseStamped
import cv2
from cv_bridge import CvBridge
import numpy as np
from fishstatecontroller.msg import State, Position

class ObjectTracker():
    def __init__(self):
        self.image = np.zeros((240,320,3), np.uint8)
        #self.subsample_ratio = 0.25             # amount to shrink image, maintains aspect ratio
        #HARDCODED - for the 320x240 res
        self.focal_length = 158.251               #the focal length of the camera
        self.real_height = 3.1                 #the real height of the target object (cm)
        self.image_center = (156.996, 111.74)    #the image center of the camera

        self.pose = PoseStamped()
        self.pose_pub = rospy.Publisher('fish_pose', PoseStamped, queue_size=10)
        self.mask_pub = rospy.Publisher('color_mask', Image, queue_size=10)
        self.found_pub = rospy.Publisher('target_found', Bool, queue_size=10)
        self.average_heading_pub = rospy.Publisher('average_heading', Float64, queue_size=10)
        self.average_pitch_pub = rospy.Publisher('average_pitch', Float64, queue_size=10)
        self.average_dist_pub = rospy.Publisher('average_dist', Float64, queue_size=10)
	self.img_pub = rospy.Publisher('grey', Image, queue_size=10)

        # Initiate position msg instance and new publisher for data
        #self.position_msg = Position()
        #self.position_pub = rospy.Publisher('fish_position', Position, queue_size=10)

        # red stretches 2 bands in hsv
        # these values are for yellow, keeping the 2 bands for red in the future
        self.hsv_lower_lower = (0,205,100)
        self.hsv_lower_upper = (35,255,255)
        self.hsv_upper_lower = self.hsv_lower_lower
        self.hsv_upper_upper = self.hsv_lower_upper

        ###create a blob tracker###
        params = cv2.SimpleBlobDetector_Params()
        
        # Change thresholds
        params.minThreshold = 10
        params.maxThreshold = 220
	params.thresholdStep = 105

        # Filter by Area.
        params.filterByArea = True
        params.minArea = 150
        params.maxArea = 50000

        # Filter by Circularity
        params.filterByCircularity = False
        params.minCircularity = 0.7

        # Filter by Convexity
        params.filterByConvexity = False
        params.minConvexity = 0.5

        # Filter by Inertia
        params.filterByInertia = False
        params.minInertiaRatio = 0.1

	#Filter by Color
	params.filterByColor = True
	params.blobColor = 255

        self.detector = cv2.SimpleBlobDetector_create(params)

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
        #img_small = cv2.resize(image, None, fx=self.subsample_ratio, fy=self.subsample_ratio, interpolation=cv2.INTER_LINEAR)
        img_blur = cv2.GaussianBlur(image, (5,5), 0)
        hsv_blur = cv2.cvtColor(img_blur, cv2.COLOR_BGR2HSV)
        mask_l = cv2.inRange(hsv_blur, self.hsv_lower_lower, self.hsv_lower_upper)
        mask_u = cv2.inRange(hsv_blur, self.hsv_upper_lower, self.hsv_upper_upper)
        mask = cv2.bitwise_or(mask_l, mask_u)

        # Get rid of noise
        kernel = np.ones((5,5),np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # Get rid of small holes
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        #Publish the mask
        mask_bgr8 = cv2.cvtColor(mask,cv2.COLOR_GRAY2BGR)
        bridge = CvBridge()
        img_masked = image
        img_masked[mask==0,:] = [0,0,0]
        grey = cv2.cvtColor(img_masked, cv2.COLOR_BGR2GRAY)
        grey_msg = bridge.cv2_to_imgmsg(grey, encoding='mono8')
        #self.img_pub.publish(cv_grey)
        
	keypoints = self.detector.detect(mask)
        orig_keypoints = list(keypoints)
        keypoints.sort(reverse=True, key=lambda k: k.size)

        # filter on size
        if len(keypoints) > 1:
            keypoints = list(filter(lambda k: k.size/keypoints[0].size > 0.4, keypoints))

            keypoints.sort(reverse=True, key=lambda k: k.pt[1])
            keypoints = [keypoints[0]]
            
        grey_with_keypoints = cv2.drawKeypoints(grey, orig_keypoints, np.array([]),(0,0,255),cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        grey_with_keypoints = cv2.drawKeypoints(grey_with_keypoints, keypoints, np.array([]),(0,255,0),cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        greyk_msg = bridge.cv2_to_imgmsg(grey_with_keypoints, encoding='bgr8')
        self.img_pub.publish(greyk_msg)

	cv_mask = bridge.cv2_to_imgmsg(mask_bgr8, encoding='bgr8')
        self.mask_pub.publish(cv_mask)

        #find the largest contour of the mask or return False,None if target is not there
        #cnts, cnt_hier = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2:]
        #if len(cnts) == 0:
        #  return (False, None, None, None)
        #cnt = max(cnts, key=cv2.contourArea)
        #((x,y),radius) = cv2.minEnclosingCircle(cnt)

        if len(keypoints) >= 1:
            keypoint = keypoints[0]
            ((x,y),radius) = (keypoint.pt, keypoint.size/2.0)
        
            if radius < 5:
                return (False, None, None, None)

            target_centroid = ((int(x),int(y)),int(radius))
            target_found = True
        else:
            return (target_found, target_centroid, 0.0, (0.0, 0.0))

        #Calculate the object's distance and offset from center
        height_px =  2.0 * target_centroid[1]
        offset_px = -1.0*(target_centroid[0][0] - self.image_center[0]) , -1.0*(target_centroid[0][1] - self.image_center[1])
        distance = (self.focal_length * self.real_height) / height_px
        y_offset = (offset_px[0] * distance)/self.focal_length
        z_offset = (offset_px[1] * distance)/self.focal_length
        heading_rad = atan2(y_offset, distance)     #heading in radians
        #print("HEADING", heading_rad)
        pitch_rad = atan2(z_offset, distance)       #pitch in radians

        # Publish distance and offset information to message file instead
        #self.position_msg.distance = str(distance)
        #self.position_msg.x_offset = str(z_offset)
        #self.position_msg.y_offset = str(y_offset)
        #self.position_pub.publish(self.position_msg)

        #return (target_found, target_centroid, distance, (y_offset, z_offset))
        return (target_found, target_centroid, distance, (heading_rad, pitch_rad))

    def run(self):
        """Run the Distance Tracker with the input from the camera"""
        bridge = CvBridge()
        rate = rospy.Rate(24)

        #for heading averaging
        current_slope = 0.0
        last_point = 0.0
        current_point = 0.0
        average = 0.0
        max_ = 0.0
        min_ = 0.0

        while not rospy.is_shutdown():
            target_found, target_centroid, dist, offset = self.find_target()
            #print("EST DISTANCE: " + str(dist) + ' cm')
            #print("EST OFFSET: " + str(offset) + ' cm')
            #print("--------------")
            if target_found:
                ###heading averaging###
                last_point = current_point
                current_point = offset[0]
                new_slope = (current_point - last_point)

                if new_slope != 0:
                    new_slope = new_slope / abs(new_slope)

                #if current slope is positive
                if current_slope == 1:
                    if new_slope <= 0:
                        average = (max_ + min_) / 2.0
                        min_ = max_
                    elif new_slope > 0:
                        max_ = current_point
                #if current slope is negative
                elif current_slope == -1:
                    if new_slope >= 0:
                        average = (max_ + min_) / 2.0
                        max_ = min_
                    elif new_slope < 0:
                        min_ = current_point
                #if current slope is zero
                else:
                    if new_slope == 0:
                        average = max_
                    elif new_slope > 0:
                        max_ = current_point
                    elif new_slope < 0:
                        min_ = current_point

                current_slope = new_slope
                #TODO pitch averaging
                #TODO dist averaging

                #Publish everything
                self.pose.header.seq = 1
                self.pose.header.stamp = rospy.Time.now()
                self.pose.header.frame_id = "sofi_cam"
                self.pose.pose.position.x = dist
                self.pose.pose.position.y = offset[0]
                self.pose.pose.position.z = offset[1]
                self.pose.pose.orientation.x = 0
                self.pose.pose.orientation.y = 0
                self.pose.pose.orientation.z = 0
                self.pose.pose.orientation.w = 1

                self.pose_pub.publish(self.pose)
                self.found_pub.publish(True)
                self.average_heading_pub.publish(average)
                self.average_pitch_pub.publish(offset[1])
                self.average_dist_pub.publish(dist)

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
    print("Beginning position tracker at 24hz\n")
    tracker.run()
    print("\ndone\n")

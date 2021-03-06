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
from std_msgs.msg import String, Float64, Bool, Header
from sensor_msgs.msg import Image, CompressedImage, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovariance, PoseWithCovarianceStamped, TwistWithCovarianceStamped
import cv2
from cv_bridge import CvBridge
import numpy as np
from fishstatecontroller.msg import State, Position

class ObjectTracker():
    def __init__(self):
        self.image = np.zeros((308,410,3), np.uint8)
        self.subsample_ratio = 0.25             # amount to shrink image, maintains aspect ratio
        self.focal_length = 318.27               #the focal length of the camera
        self.real_height = 2.0                 #the real height of the target object
        self.image_center = (203.55, 150.58)    #the image center of the camera

	self.imu_msg = None
        self.imu_pub = rospy.Publisher('/imu/data', Imu, queue_size=10)
        self.twist_msg = None
        self.twist_pub = rospy.Publisher('/fish_twist', TwistWithCovarianceStamped, queue_size=10)
        self.pose = PoseWithCovarianceStamped() #TODO replace with Odometry (?)
        self.pose_pub = rospy.Publisher('fish_pose', PoseWithCovarianceStamped, queue_size=10)
        self.mask_pub = rospy.Publisher('color_mask', Image, queue_size=10)
        self.found_pub = rospy.Publisher('target_found', Bool, queue_size=10)
        self.average_heading_pub = rospy.Publisher('average_heading', Float64, queue_size=10)
        self.average_pitch_pub = rospy.Publisher('average_pitch', Float64, queue_size=10)
        self.average_dist_pub = rospy.Publisher('average_dist', Float64, queue_size=10)

        # Initiate position msg instance and new publisher for data
        #self.position_msg = Position()
        #self.position_pub = rospy.Publisher('fish_position', Position, queue_size=10)

        # red stretches 2 bands in hsv
        # these values are for yellow, keeping the 2 bands for red in the future
        self.hsv_lower_lower = (14,85,55)
        self.hsv_lower_upper = (30,255,235)
        self.hsv_upper_lower = self.hsv_lower_lower
        self.hsv_upper_upper = self.hsv_lower_upper

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
        height_px =  2.0 * target_centroid[1]
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
        rate = rospy.Rate(24)

        #for heading averaging
        current_sum = 0.0
        count = 0.0
        current_slope = 0.0
        last_point = 0.0
        current_point = 0.0
        average = 0.0

        while not rospy.is_shutdown():
            target_found, target_centroid, dist, offset = self.find_target()
            #print("EST DISTANCE: " + str(dist) + ' inches')
            #print("EST OFFSET: " + str(offset) + ' inches')
            #print("--------------")
            if target_found:
                print("Found.")
                #heading averaging
                last_point = current_point
                current_point = offset[0]
                new_slope = (current_point - last_point)
                if new_slope != 0:
                    new_slope = new_slope / abs(new_slope)
                if current_slope == 1:
                    if new_slope < 0:
                        average = current_sum / count
                        current_sum = 0.0
                        count = 0.0
                    #print("slope is 1")
                current_sum += current_point
                count += 1
                #slope can never be 0 (kind of hacky solution to the situation where
                # 'current_slope' is 0 and 'new_slope' is negative, which would cause
                # the program to not take the average)
                if new_slope != 0:
                    current_slope = new_slope
                #TODO pitch averaging
                #TODO dist averaging

                #IMU + Camera data fusion
                #NOTE: All sensor data should in "sofi_cam" frame for now

                #IMU
                #1) Convert euler rates to linear velocities & publish separately
                #2) Publish original Imu message
                if self.imu_msg is not None:
                    self.twist_msg = TwistWithCovarianceStamped()
                    h = Header()
                    h.stamp = rospy.Time.now()
                    h.frame_id = "base_link" #TODO convert to base_link frame
                    self.twist_msg.header = h

                    #Compute cross product of IMU angular velocities and distance to object to estimate object's tangential velocity
                    omega = [-self.imu_msg.angular_velocity.x, -self.imu_msg.angular_velocity.y, -self.imu_msg.angular_velocity.z]
                    r = [dist, offset[0], offset[1]]
                    v_tan = np.cross(omega, r)

                    #Publish TwistWithCovarianceStamped message
                    self.twist_msg.twist.twist.linear.x = v_tan[0]
                    self.twist_msg.twist.twist.linear.y = v_tan[1]
                    self.twist_msg.twist.twist.linear.z = v_tan[2]
                    self.twist_msg.twist.twist.angular.x = 0
                    self.twist_msg.twist.twist.angular.y = 0
                    self.twist_msg.twist.twist.angular.z = 0
                    self.twist_msg.twist.covariance = [0.003, 0, 0, 0, 0, 0, 0, 0.003, 0, 0, 0, 0, 0, 0, 0.003, 0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0.1] #TODO determine covariance matrix
                    self.twist_pub.publish(self.twist_msg)

                    #Publish Imu message (after flipping acceleration sign)
                    self.imu_msg.linear_acceleration.x = -self.imu_msg.linear_acceleration.x
                    self.imu_msg.linear_acceleration.y = -self.imu_msg.linear_acceleration.y
                    self.imu_msg.linear_acceleration.z = -self.imu_msg.linear_acceleration.z
                    self.imu_pub.publish(self.imu_msg)

                #Publish PoseWithCovarianceStamped from raspicam
                self.pose.header.seq = 1
                self.pose.header.stamp = rospy.Time.now()
                self.pose.header.frame_id = "odom"

                p = PoseWithCovariance()
                p.pose.position.x = dist
                p.pose.position.y = offset[0]
                p.pose.position.z = offset[1]
                p.pose.orientation.x = 0
                p.pose.orientation.y = 0
                p.pose.orientation.z = 0
                p.pose.orientation.w = 1
                p.covariance = [0.003, 0, 0, 0, 0, 0, 0, 0.003, 0, 0, 0, 0, 0, 0, 0.003, 0, 0, 0, 0, 0, 0, 0.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.0] #TODO determine covariance matrix

                self.pose.pose = p

                self.pose_pub.publish(self.pose)
                self.found_pub.publish(True)
                self.average_heading_pub.publish(average)
                self.average_pitch_pub.publish(offset[1])
                self.average_dist_pub.publish(dist)

            else:
                self.found_pub.publish(False)

                #TODO IMU only (?)

            rate.sleep()

    def callback(self, ros_data):
        bridge = CvBridge()
        self.image = bridge.compressed_imgmsg_to_cv2(ros_data, desired_encoding='passthrough')

    def imu_cb(self, ros_data):
        self.imu_msg = ros_data

if __name__ == '__main__':
    rospy.init_node('state_estimation', anonymous=True)
    tracker = ObjectTracker()
    rospy.Subscriber('/raspicam_node/image/compressed', CompressedImage, tracker.callback)
    rospy.Subscriber('/imu/data/raw', Imu, tracker.imu_cb)
    print("Beginning position tracker at 24hz\n")
    tracker.run()
    print("\ndone\n")

#!/usr/bin/python
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage

rospy.init_node("hsv_tool")

img_transport = "normal"

bridge = CvBridge()
img = None
hsv = None
mask = None

hue_thresh = 10
val_thresh = 10
hsv_upper = (60,160,160)
hsv_lower = (20,30,80)

def img_callback(data):
    global img
    global hsv
    global mask
    if img_transport == "compressed":
        img = bridge.compressed_imgmsg_to_cv2(data, "bgr8")
    else:
        img = bridge.imgmsg_to_cv2(data, "bgr8")

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, hsv_lower, hsv_upper)

    if np.any(mask):
        img[mask>0,1] = 255

def mouse_callback(event, x, y, flags, param):
    global img
    global hsv

    if event == cv2.EVENT_LBUTTONUP:
        print("hsv: ",hsv[y,x])

# register image callback
img_topic = rospy.get_param("~image_topic", "/raspicam_node/image")
img_transport = rospy.get_param("~image_transport", "compressed")
rate = rospy.get_param("~rate",60)

if img_transport == "compressed":
    rospy.Subscriber(img_topic+"/compressed", CompressedImage, img_callback)
else:
    rospy.Subscriber(img_topic, CompressedImage, img_callback)


ros_rate = rospy.Rate(rate)    

while not rospy.is_shutdown():
    if img is not None:
        cv2.imshow("cam", img)
        cv2.setMouseCallback("cam", mouse_callback, 0)
        cv2.waitKey(1)
    ros_rate.sleep()

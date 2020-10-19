#!/usr/bin/env python

#Calibration Parser
# Retrieves and stores info about the raspberry pi cam in two ways:
#   - via a CamInfoContainer object that retrieves cam info from 
#     the /raspicam_node/camera_info topic
#   - via the calibration_parser function that retrieves cam info 
#     from a selected calibration file (YAML file with a specific format)

import yaml
import rospy
import roslib
from sensor_msgs.msg import CameraInfo

class CamInfoContainer():
    """Stores camera info for later use"""    
    def __init__(self):    
        self.width = 0
        self.height = 0
        self.focal_length = 0
        self.camera_center = 0,0

    def set_camera_info(self, W, H, F, C):
        """sets all the camera parameters"""
        self.width = W
        self.height = H
        self.focal_length = F
        self.camera_center = C

    def get_camera_info(self):
        return (self.width, self.height, self.focal_length, self.camera_center)

    def callback(self, rosdata):
        self.set_camera_info(rosdata.width, rosdata.height, rosdata.K[4], (rosdata.K[2], rosdata.K[5]))

    def run(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            info = self.get_camera_info()
            for item in info:
                print(item)
            print("----")
            rate.sleep()
 
def calibration_parser(fname="/home/bncook/.ros/camera_info/camerav2_1280x960.yaml"):
    with open(fname) as file:
        calibration_data = yaml.load(file, Loader=yaml.FullLoader)
        width = calibration_data['image_width']
        height = calibration_data['image_height']
        focal_length = calibration_data['camera_matrix']['data'][4]
        camera_center = calibration_data['camera_matrix']['data'][2] , calibration_data['camera_matrix']['data'][5]
        print(width)
        print(height)
        print(focal_length)
        print(camera_center)
    return 0

if __name__ == '__main__':
    print("Calibration parser results")    
    calibration_parser()
    rospy.init_node('cam_info_parser', anonymous=True)
    container = CamInfoContainer()
    rospy.Subscriber('/raspicam_node/camera_info', CameraInfo, container.callback)
    print("CamInfoContainer results")
    container.run()
    print("\ndone\n")

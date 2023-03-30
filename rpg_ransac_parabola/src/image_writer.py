#!/usr/bin/env python3

'''
Runs a RANSAC to determine the parabola along which the projectile will move + tracks this impact point with a naive
Kalman Filter
'''
# standard
import os
import sys

# ROS
import queue
import roslib 
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# other third-party
import cv2
import matplotlib.pyplot as plt
import numpy as np

# custom
from utils import load_bboxes, filter_to_max_bboxes, filter_bboxes_to_interval
from fit_parabola import fit_parabola_to_bboxes, compute_impact_point

class Recorder:
    '''
    Takes in the center of the largest cluster on an image and tracks it using RANSAC + KF. Publishes the ball state @ 100 Hz.
    '''
    def __init__(self):
        self.cluster_center_sub = rospy.Subscriber("/sees/cluster_image", Image, self.process_measurement)
        self.img_idx = 0
        self.bridge = CvBridge()
    
    def process_measurement(self, img):
        '''
        Processes an incoming cluster image
        '''
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img, "bgr8")
            cv2.imwrite(f"/home/bforrai/catkin_ws/src/bounding_box_parabola_fitting/src/thresh_robot/thresh_image_{self.img_idx}.png", cv_image)
            self.img_idx += 1
            print("Img written")
        except CvBridgeError as e:
            print(e)

if __name__ == '__main__':
    rospy.loginfo("Node init...")
    ransac_tracker = Recorder()
    rospy.init_node('ransac_recroder', anonymous=True)
    rospy.loginfo("Node init ran")
    rospy.spin()

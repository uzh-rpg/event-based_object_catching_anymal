#!/usr/bin/env python3

'''
Records bounding box messages to a log .txt file
'''
# standard
import os
import sys

# ROS
import rospy
from rpg_dynamic_obstacle_avoidance_msgs.msg import BoundingBox2DStamped


class BBoxPrinter:
    '''
    Takes in the center of the largest cluster on an image and tracks it using RANSAC. Publishes the ball state @ 100 Hz.
    '''
    def __init__(self):
        self.cluster_center_sub = rospy.Subscriber("/sees/cluster_bbox", BoundingBox2DStamped, self.process_measurement)
            
    def process_measurement(self, bbox_msg):
        '''
        Prints the incoming message in a format Daniel used
        '''
        # test if area is OK
        if (bbox_msg.bbox.size_y * bbox_msg.bbox.center.x):
            print(f"{bbox_msg.header.stamp} {bbox_msg.bbox.size_x} {bbox_msg.bbox.size_y} {bbox_msg.bbox.center.x} {bbox_msg.bbox.center.y} {bbox_msg.bbox.center.theta} {bbox_msg.image_idx}")

if __name__ == '__main__':
    bbox_printer = BBoxPrinter()
    rospy.init_node('ransac_bbox_printer', anonymous=True)
    rospy.spin()

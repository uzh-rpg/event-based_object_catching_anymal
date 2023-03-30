#!/usr/bin/env python3

'''
ROS Node that runs a RANSAC to determine the parabola along which the projectile will move, and calculates the impact point.
Subscribes to:
- /sees/cluster_bbox: BoundingBox2DStamped: the center of the largest cluster on an image

Publishes:
- /sees/impact_marker_array: MarkerArray: the impact points
- /sees/inlier_marker_array: MarkerArray: the inlier points of the RANSAC
- /sees/outlier_marker_array: MarkerArray: the outlier points of the RANSAC
- /sees/parabola_marker: Marker: a visualization of the trajectory
- /sees/impact_marker: Marker: a visualization of the morst recent impact point
- /sim_capture/ball_states: Float32MultiArray: net center position command to our robot @ 100 Hz (can be disabled for new applications)
'''
# standard
import os
import sys
import time 

# ROS
import queue
import roslib 
import rospy
from std_msgs.msg import  Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped, Point

# other third-party
import matplotlib.pyplot as plt
import numpy as np

# custom
from utils import load_bboxes, filter_to_max_bboxes, filter_bboxes_to_interval
from fit_parabola import fit_parabola_to_bboxes, compute_impact_point, sample_parabola

DEBUG_NODE = True

class RansacTracker:
    '''
    Takes in the center of the largest cluster on an image and tracks it using RANSAC + KF. Publishes the ball state @ 100 Hz.
    '''
    def __init__(self):
        self.cluster_center_sub = rospy.Subscriber("/sees/cluster_bbox", PointStamped, self.process_measurement)
        self.ball_state_pub = rospy.Publisher("/sim_capture/ball_states", Float32MultiArray, queue_size = 1)
        self.impact_array_pub = rospy.Publisher("/sees/impact_marker_array", MarkerArray, queue_size = 1)
        self.inlier_array_pub = rospy.Publisher("/sees/inlier_marker_array", MarkerArray, queue_size = 1)
        self.outlier_array_pub = rospy.Publisher("/sees/outlier_marker_array", MarkerArray, queue_size = 1)
        self.parabola_pub = rospy.Publisher("/sees/parabola_marker", Marker, queue_size = 1)
        self.impact_pub = rospy.Publisher("/sees/impact_marker", Marker, queue_size = 1)
        self.points_4d = []
        self.intersection_markers = []
        self.impact_points = []
        # rolling buffer of bboxes to consider for parabola fitting
        self.bboxes_buffer_len = 50
        # discard bboxes older than 0.5 secs from buffer
        self.time_window_oldest = 2.5
        # RANSAC discards bboxes for parabola fitting, which are 0.2m away from the parabola
        self.inlier_threshold = 0.25
        self.t_step = 0.01
        self.t_since_throw = 0.0
        self.max_skipped_detections = 5
        self.skipped_detection_num = 0
        self.marker_idx = 0
        self.ball_pub_rate = 100
        self.marker_array = MarkerArray()
        self.start_time_set = False
        self.use_z_only = True
        self.point_distance_threshold = 0.05

    def init_filter(self):
        '''
        Reinitializes the Kalman Filter that is used to track the projectiles.
        '''
        self.t_since_throw = 0.0
        self.skipped_detection_num = 0
        self.points_4d = []
        self.impact_points = []
        self.delete_marker_array(self.inlier_array_pub)
        self.delete_marker_array(self.outlier_array_pub)
        self.delete_marker_array(self.impact_array_pub)
        self.marker_array.markers = []
        
    def process_measurement(self, point_msg):
        '''
        Processes an incoming cluster center 3D point
        '''
        print("Got measurement, x: ", point_msg.point.x, " y: ", point_msg.point.y, " z: ", point_msg.point.z)
        POINT_CLOUD_MAX_Z = 8        
        if point_msg.point.z > POINT_CLOUD_MAX_Z or point_msg.point.z < 0:
            #print("Skipping 3D point, too far away")
            self.skipped_detection_num += 1

            if self.skipped_detection_num >= self.max_skipped_detections:
                # rospy.loginfo(f"More than {self.max_skipped_detections} skipped detections, reinitializing...")
                self.init_filter()
                self.skipped_detection_num = 0
        else:
            time = self._get_time_from_stamp(point_msg.header.stamp)

            point_4d = np.array([point_msg.point.x, point_msg.point.y, point_msg.point.z, time]).astype("float32")
            if len(self.points_4d):
                points_4d_to_process = np.stack(self.points_4d)
                distance_vectors = points_4d_to_process[:,:3] - point_4d[:3].reshape(1,3)
                # calculate min
                if self.use_z_only:
                    distance_vectors = distance_vectors[:,2].reshape(-1,1)
                distances = np.linalg.norm(distance_vectors, axis=1)
                min_dist = np.min(distances)
                if min_dist > self.point_distance_threshold:
                    self.points_4d.append(point_4d)
                    self.skipped_detection_num = 0
                    rospy.loginfo(f"Detected object at {point_4d}")
                    rospy.loginfo(f"Num 3D points in buffer = {len(self.points_4d)}")
                else:
                    self.skipped_detection_num += 1
                    rospy.logwarn('Non-sufficient distance from previous points: Skipping detection')
                if len(self.points_4d) > self.bboxes_buffer_len:
                    self.points_4d.pop(0)
            else:
                self.points_4d.append(point_4d)
                self.skipped_detection_num = 0
                rospy.loginfo(f"Detected object at {point_4d}")
                rospy.loginfo(f"Num 3D points in buffer = {len(self.points_4d)}")

    def publish_ball_state(self, _ = None):
        '''
        Publishes the ball state & the intersection point as a Float32MultiArray message
        '''
        if len(self.points_4d) < 2:
            self._publish_ball_state_msg(0, -0.92, 0)
            return 

        start = time.time()
        points_4d_to_process = np.stack(self.points_4d)
        initial_position, initial_velocity, inliers, points_3d = fit_parabola_to_bboxes(points_4d_to_process, inlier_threshold=self.inlier_threshold)

        # print("initial position")
        if np.isnan(initial_velocity[0]):
            self._publish_ball_state_msg(0, -0.92, 0)
            return 

        # print("impact point")
        impact_point = compute_impact_point(initial_position, initial_velocity)
        rospy.loginfo(f"Impact Point at Px={impact_point[0]} Py={impact_point[1]}. Publishing msg. Running @ {1/(time.time()-start)} Hz")

        self.impact_points.append(impact_point)
        median_impact_point = np.median(np.stack(self.impact_points), axis=0)
        self._publish_ball_state_msg(median_impact_point[0], median_impact_point[1])

        if DEBUG_NODE:
            # print("Publishing markers")
            self._get_impact_marker_array(impact_point)
            self._get_inlier_outlier_marker_arrays(inliers, points_3d)
            self._get_median_impact_point(median_impact_point)
            self._get_parabola_marker_array(initial_velocity, initial_position)

    def _publish_ball_state_msg(self,cam_x,cam_y, cam_z):
        '''
        Publishes the ball state to a Float32MultiArray topic
        '''
        msg = Float32MultiArray()
        msg.data = (0.0, -cam_x, cam_y)
        self.ball_state_pub.publish(msg)

    def _get_parabola_marker_array(self, initial_velocity, initial_position):
        """
        Publishes parabola as line segments
        """
        #self.delete_marker_array(self.parabola_array_pub)
        marker = Marker()
        marker.header.frame_id = "dvs"
        marker.type = marker.LINE_STRIP
        marker.action = marker.ADD
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        t_max = - initial_position[2] / initial_velocity[2]
        points = sample_parabola(np.linspace(0, t_max, 20), initial_position, initial_velocity)
        for i, (px, py, pz) in enumerate(points):
            p = Point()
            p.x = px 
            p.y = py 
            p.z = pz 
            marker.points.append(p)

        self.parabola_pub.publish(marker)

    def _get_inlier_outlier_marker_arrays(self, inliers, points_3d):
        '''
        Publishes MarkerArrays from the inliers and outliers of the RANSAC.
        '''
        # delete both arrays to refresh
        self.delete_marker_array(self.inlier_array_pub)
        self.delete_marker_array(self.outlier_array_pub)
        # collect inliers and outliers
        inlier_marker_array = MarkerArray()
        outlier_marker_array = MarkerArray()

        for i in range(points_3d.shape[0]):
            if not np.any(np.isnan(points_3d[i,:])):
                ball_point=PointStamped()
                ball_point.header.frame_id = "dvs"
                ball_point.header.stamp =rospy.Time(0)
                ball_point.point.x = points_3d[i,0]
                ball_point.point.y = points_3d[i,1]
                ball_point.point.z = points_3d[i,2]
                marker = Marker()
                marker.header.frame_id = "dvs"
                marker.type = marker.SPHERE
                marker.action = marker.ADD
                marker.scale.x = 0.1
                marker.scale.y = 0.1
                marker.scale.z = 0.1
                marker.id = i
                marker.pose.orientation.w = 1.0
                marker.pose.position.x = ball_point.point.x
                marker.pose.position.y = ball_point.point.y 
                marker.pose.position.z = ball_point.point.z
                
                if inliers[i]:
                    marker.color.a = 1.0
                    marker.color.r = 0.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0
                    inlier_marker_array.markers.append(marker)
                    
                else:
                    marker.color.a = 1.0
                    marker.color.r = 1.0
                    marker.color.g = 0.0
                    marker.color.b = 0.0
                    outlier_marker_array.markers.append(marker) 
        
        self.inlier_array_pub.publish(inlier_marker_array)
        self.outlier_array_pub.publish(outlier_marker_array)

    def delete_marker_array(self, publisher):
        '''
        Deletes all of the markers published by publisher
        '''
        marker_array = MarkerArray()
        marker = Marker()
        marker.id = 0
        marker.action = Marker.DELETEALL
        marker_array.markers.append(marker)
        publisher.publish(marker_array)

    def _publish_ransac_debug_img(self):
        '''
        Publishes a similar debug image to Daniel's code
        '''

    def _get_impact_marker_array(self, impact_point):
        '''
        Returns the Markerarray that displays the impact points.
        '''
        ball_point=PointStamped()
        ball_point.header.frame_id = "dvs"
        ball_point.header.stamp =rospy.Time(0)
        ball_point.point.x = 0.0
        ball_point.point.y = impact_point[0] 
        ball_point.point.z = -impact_point[1]
        marker = Marker()
        marker.header.frame_id = "dvs"
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.id = self.marker_idx
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = ball_point.point.x
        marker.pose.position.y = ball_point.point.y 
        marker.pose.position.z = ball_point.point.z
        # We add the new marker to the MarkerArray, removing the oldest marker from it when necessary
        self.marker_array.markers.append(marker) 
        self.impact_array_pub.publish(self.marker_array)
        self.marker_idx += 1

    def _get_median_impact_point(self, impact_point):
        marker = Marker()
        print("publishing impact pt")
        marker.header.frame_id = "odom"

        marker.type = marker.CUBE
        marker.action = marker.ADD
        marker.scale.x = 0.15
        marker.scale.y = 0.15
        marker.scale.z = 0.15
        marker.color.a = 1.0

        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0

        marker.id = self.marker_idx
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = impact_point[0]
        marker.pose.position.y = impact_point[1] 
        marker.pose.position.z = impact_point[2]
        print("Impact point ", impact_point)
        self.impact_pub.publish(marker)

    def _get_time_from_stamp(self, timestamp):
        '''
        Gives the elapsed seconds as a float from a ros timestamp
        '''
        if self.start_time_set:
            time = float(timestamp.nsecs)/10e9 + float(timestamp.secs) - self.start_time
        else:
            self.start_time = float(timestamp.nsecs)/10e9 + float(timestamp.secs)
            time = 0.0
            self.start_time_set = True
        return time

if __name__ == '__main__':
    rospy.loginfo("Node init...")
    ransac_tracker = RansacTracker()
    rospy.init_node('ransac_bbox_tracker', anonymous=True)
    rospy.Timer(rospy.Duration(1.0/ransac_tracker.ball_pub_rate), ransac_tracker.publish_ball_state)
    rospy.loginfo("Node init ran")
    rospy.spin()

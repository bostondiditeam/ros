#!/usr/bin/env python

from __future__ import print_function
from __future__ import division
import rospy, rosbag, tf
from geometry_msgs.msg import Point, Quaternion
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray, InteractiveMarker,  InteractiveMarkerControl, InteractiveMarkerFeedback
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
import cv2, cv_bridge
from image_geometry import PinholeCameraModel
import numpy as np
import csv, sys, os, copy
from collections import defaultdict
import PyKDL as kd
from camera_info import *
from utils import *
from parse_tracklet import *
import argparse
import Queue

bridge = cv_bridge.CvBridge()

class Projection:
    def __init__(self, calib_file):
        self.calib_file = calib_file
        self.imgOutput = rospy.Publisher('/image_bbox', Image, queue_size=1)

        # camera image
        self.camera_image = None
        self.mv3d_bbox = []
        self.kalman_bbox = []

    def subscribe_messages(self):
        # image_raw_sub = message_filters.Subscriber('/image_raw', Image)
        # bbox_sub = message_filters.Subscriber('/bbox', MarkerArray)
        # ts = message_filters.ApproximateTimeSynchronizer([image_raw_sub, bbox_sub], 3, 0.03)
        # ts.registerCallback(self.handle_msg)
        rospy.Subscriber('/image_raw', Image, self.handle_img_msg)
        rospy.Subscriber('/bbox', MarkerArray, self.handle_bbox_msg)
        rospy.Subscriber('/bbox_final', MarkerArray, self.handle_bbox_final_msg)


    def _draw_marker_to_image(self, out_img, m):
        dims = np.array([m.scale.x, m.scale.y, m.scale.z])
        obs_centroid = np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z])
        orient = np.array([m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w])

        if obs_centroid is None:
            rospy.loginfo("Couldn't find obstacle centroid")
            return out_img

        # print centroid info
        rospy.loginfo(str(obs_centroid))

        # case when obstacle is not in camera frame
        if obs_centroid[0] < 3:
            return out_img
            # self.imgOutput.publish(bridge.cv2_to_imgmsg(img, 'bgr8'))
            # return

        # get bbox
        R = tf.transformations.quaternion_matrix(orient)
        corners = [0.5 * np.array([i, j, k]) * dims for i in [-1, 1]
                   for j in [-1, 1] for k in [-1, 1]]
        corners = [obs_centroid + R.dot(list(c) + [1])[:3] for c in corners]
        cameraModel = PinholeCameraModel()
        cam_info = load_cam_info(self.calib_file)
        cameraModel.fromCameraInfo(cam_info)
        projected_pts = [cameraModel.project3dToPixel(list(pt) + [1]) for pt in corners]
        projected_pts = np.array(projected_pts)
        center = np.mean(projected_pts, axis=0)
        color = (m.color.b*255, m.color.g*255, m.color.r*255)
        out_img = drawBbox(out_img, projected_pts, color=color)
        return out_img

    def _draw_bblox_image(self):
        out_img = self.camera_image

        for m in self.mv3d_bbox:
            out_img = self._draw_marker_to_image(out_img, m)

        for m in self.kalman_bbox:
            out_img = self._draw_marker_to_image(out_img, m)

        self.imgOutput.publish(bridge.cv2_to_imgmsg(out_img, 'bgr8'))

    def handle_bbox_msg(self, bbox_msg):
        if self.camera_image == None:
            return
        self.mv3d_bbox = bbox_msg.markers
        self._draw_bblox_image()

    def handle_bbox_final_msg(self, bbox_final_msg):
        if self.camera_image == None:
            return
        self.kalman_bbox = bbox_final_msg.markers
        self._draw_bblox_image()

    def handle_img_msg(self, img_msg):
        timestamp = img_msg.header.stamp.to_nsec()
        print('=====image_callback: msg : seq=%d, timestamp=%19d' % (img_msg.header.seq, timestamp))
        self.camera_image = bridge.imgmsg_to_cv2(img_msg, 'bgr8')

if __name__ == "__main__" :
    parser = argparse.ArgumentParser(description="visulaize for result")
    parser.add_argument('calib', type=str, nargs='?', default='', help='calibration filename')
    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node('projection')

    calib_file = args.calib
    assert os.path.isfile(calib_file), 'Calibration file %s does not exist' % calib_file

    try :
        p = Projection(calib_file)
        p.subscribe_messages()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


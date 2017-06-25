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
        self.img_msg_queue = list() # Queue.queue()
        self.camera_image = None

    def subscribe_messages(self):
        # image_raw_sub = message_filters.Subscriber('/image_raw', Image)
        # bbox_sub = message_filters.Subscriber('/bbox', MarkerArray)
        # ts = message_filters.ApproximateTimeSynchronizer([image_raw_sub, bbox_sub], 3, 0.03)
        # ts.registerCallback(self.handle_msg)
        rospy.Subscriber('/image_raw', Image, self.handle_img_msg)
        rospy.Subscriber('/bbox', MarkerArray, self.handle_bbox_msg)

    def handle_bbox_msg(self, bbox_msg):
        if self.camera_image == None: return
        markers = bbox_msg.markers
        out_img = self.camera_image
        if len(markers) > 0:
            seq = markers[0].header.seq
            timestamp = markers[0].header.stamp.to_nsec()
            print('=====bbox_callback: msg : seq=%d, timestamp=%19d' % (seq, timestamp))
            # sync
            # i = 0
            # for i in range(len(self.img_msg_queue)):
            #     if timestamp < self.img_msg_queue[i].header.stamp.to_nsec():
            #         out_img = bridge.imgmsg_to_cv2(self.img_msg_queue[i], 'bgr8')
            #         self.img_msg_queue = self.img_msg_queue[i+1:]
            #         break

        for m in markers:
            dims = np.array([m.scale.x, m.scale.y, m.scale.z])
            obs_centroid = np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z])
            orient = np.array([m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w])

            if obs_centroid is None:
                rospy.loginfo("Couldn't find obstacle centroid")
                continue

            # print centroid info
            rospy.loginfo(str(obs_centroid))

            # case when obstacle is not in camera frame
            if obs_centroid[0] < 3:
                continue
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
            color = (m.color.b, m.color.g, m.color.r)
            out_img = drawBbox(out_img, projected_pts, color=color)

        self.imgOutput.publish(bridge.cv2_to_imgmsg(out_img, 'bgr8'))

    def handle_img_msg(self, img_msg):
        timestamp = img_msg.header.stamp.to_nsec()
        print('=====image_callback: msg : seq=%d, timestamp=%19d' % (img_msg.header.seq, timestamp))

        img = None
        img = bridge.imgmsg_to_cv2(img_msg, 'bgr8')
        self.img_msg_queue.append(img_msg)
        self.camera_image = img

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


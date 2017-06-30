#!/usr/bin/env python

import rospy
import numpy as np
import tf
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import PointCloud2, Image
from Tracklet_saver import *
import argparse

import multi_object_tracker as mot
import time
import os

log_dir = os.path.abspath(os.path.join(os.path.dirname(__file__),'dump'))
if os.path.isdir(log_dir) == False: os.makedirs(log_dir)
log_count=0

class Tracker:

    def __init__(self):
        self.current_time = rospy.Time()
        self.tracklet_saver = Tracklet_saver('./')
        self.lidar_lasttimestamp = -1
        self.tracklet_lasttimestamp = -1
        self.tracklet_generated = False
        self.cameraframeindex = -1

        # initialize Kalman Tracker
        #default params (min_hits=3, max_hist=10, distance_threshold=.03)
        self.mot_tracker = mot.Sort(min_hits=5, max_age=15,
                                    distance_threshold=.06)
        self.num_updates = 0
        self.tracked_targets = [] #defaultdict(partial(deque, maxlen=5))
        self.total_time = 0


    def write_final_tracklet_xml(self):
        if not self.tracklet_generated:
            self.tracklet_saver.write_tracklet()
            self.tracklet_generated = True
            print('---------tracklet exported------------------')
            #rospy.signal_shutdown('Tracklet exported')

    def save_gen_tracklet(self, frameid, scale, trans, rot): #, timestamp, bbox):
        #print 'Save tracklet'
        if not self.tracklet_generated:
            self.tracklet_saver.add_tracklet(frameid, scale, trans, rot)

    def kalman_update(self, frameid, scale, trans, rot):
        #print 'Kalman update #', trackid
        #########################################################
        #TODO: Kalman filter measurement update for single target
        #########################################################
        self.save_gen_tracklet(frameid, scale, trans, rot)

    def _hungarian_update_(self, markerarry):
        """This function is not used and is legacy code. preserved for reference"""
        #print 'Hungarian update'
        trackindex = 0
        bboxtime = markerarry[-1].header.stamp.to_nsec() # use last entry as time reference (should be the same for all)
        if bboxtime > self.tracklet_lasttimestamp:
            for m in markerarry:

                scale = np.asarray([m.scale.x,m.scale.y,m.scale.z])
                trans = np.asarray([m.pose.position.x,m.pose.position.y,m.pose.position.z])

                rotq = (m.pose.orientation.x,m.pose.orientation.y,m.pose.orientation.z,m.pose.orientation.w)
                euler = tf.transformations.euler_from_quaternion(rotq)
                roll = euler[0]
                pitch = euler[1]
                yaw = euler[2]
                rot = np.asarray([roll,pitch,yaw])

                if  m.header.stamp.to_nsec() == bboxtime: # in case there are old entries
                    print('bbox ',m.id, ' time = ', m.header.stamp.to_nsec(), ': Pos = [',trans,']')
                    bboxtime = m.header.stamp.to_nsec()
                    trackindex += 1

                    #########################################################
                    #TODO: multi-target tracking algorithm update
                    #########################################################
                    self.kalman_update(self.cameraframeindex, trackindex, scale, trans, rot)

            self.tracklet_lasttimestamp = bboxtime

    def tracker_update(self, detections):
        global log_count
        """Performs Kalman Filter update and maintains track on obstacles.

        Args:
            detections (:obj:`numpy.array`) : an array of detected bounding
                boxes. Each row correspond to a detection of the form
                `[tx, ty,  tz, w, l, rz, h, rx, ry]`. The order of attributes
                (columns) should be strictly followed.

        Note:
            The order of columns corresponding to bounding box attributes
            is important because same order is followed while extracting
            columns in kalman filter update method. Reason for altering
            (mangling) the column order is to make it easier to slice the
            numpy array of detections into a subset that is used by kalman
            filter `[tx, ty, tz, w, l, rz]`, and one that is not used by
            Kalman Filter `[h, rx, ry]`. The second slice does not
            play any role in Kalman Filter update, merely passed through
            to enable publishing tracked obstacle, and writing xml.

        """
        start_time = time.time()
        self.tracked_targets, self.tracked_ids = self.mot_tracker.update(detections)
        rospy.logerr('\ndetections ={}\ntype={}\n'.format(detections,type(detections)))
        rospy.logerr('\nself.tracked_targets ={}\nself.tracked_ids={}\n'.
                     format(self.tracked_targets, self.tracked_ids))
        path = os.path.join(log_dir,'detections_%05d'%(log_count))
        np.save(path, detections)
        rospy.logerr('save: {} ok'.format(path))
        log_count=log_count+1


        # the kalman filter returns numpy array in the form:
        # self.tracked_targets shape = [n_detections, box_dim]
        # and each row = [tx, ty, w, l, rz, tz, h, rx, ry]

        cycle_time = time.time() - start_time
        self.total_time += cycle_time
        self.num_updates += 1.

        self.ros_publish_final_bbox()

        print('kalman update')
        for i in range(self.tracked_targets.shape[0]):
            tx, ty, tz, w, l, rz,  h, rx, ry = self.tracked_targets[i,:]
            # print(tx, ty, w, l, rz, tz, h, rx, ry)
            scale = np.asarray([w, l, h])
            trans = np.asarray([tx, ty, tz])
            rot = np.asarray([rx, ry, rz])

            # self.kalman_update(self.cameraframeindex, scale, trans, rot)
            self.save_gen_tracklet(self.cameraframeindex, scale, trans, rot)

        print("tracking {} objects at time {}, processing time {}, avg processing time {}"
              .format(len(self.tracked_targets), self.current_time,
                      cycle_time, self.total_time/self.num_updates))

    def handle_image_msg(self, msg):
        self.cameraframeindex += 1

    def handle_lidar_msg(self, msg):
        if (not self.tracklet_generated):
            timestamplidar = msg.header.stamp.to_nsec()
            if timestamplidar < self.lidar_lasttimestamp: # rosplay loops
                print('generating tracklet')
                self.write_final_tracklet_xml()
            else:
                self.lidar_lasttimestamp = timestamplidar
                print('lidar time ', self.lidar_lasttimestamp)

    def handle_bbox_msg(self,msg):
        if (not self.tracklet_generated) and (len(msg.markers)>0):
            #print('time0', 'time1')
            #print(msg.markers[0].header.stamp.to_nsec())
            print("handle_bbox_msg", msg.markers[-1].header.stamp.to_nsec())
            self.bbox_lasttimestamp = msg.markers[-1].header.stamp

            # print 'bbox time  ', self.tracklet_lasttimestamp, ', Total bbox in this frame = ', len(msg.markers)
            bboxes = self._marker_to_boxes(msg.markers)
            # self.hungarian_update(np.asarray(bboxes))
            if 0:   #debug info
                m = msg.markers[0]
                print('bounding boxes')
                print(np.asarray(bboxes))

            self.tracker_update(np.asarray(bboxes))
            self.tracklet_lasttimestamp = msg.markers[-1].header.stamp.to_nsec()


    def _marker_to_boxes(self, markers):
        """Convert MarkerArray to bounding boxes.

        Args:
            msg (:obj:`rospy.msg`)
        """
        bboxtime = markers[-1].header.stamp.to_nsec() # use last entry as time reference (should be the same for all)
        bboxes = []

        for m in markers:
            trans = m.pose.position
            scale = m.scale
            tx, ty, tz = trans.x, trans.y, trans.z
            l, w, h = scale.x, scale.y, scale.z
            rx, ry, rz = tf.transformations.euler_from_quaternion(
                [m.pose.orientation.x,
                 m.pose.orientation.y,
                 m.pose.orientation.z,
                 m.pose.orientation.w])

            # if m.header.stamp.to_nsec() == bboxtime:
            #if (m.header.stamp.to_nsec() > self.tracklet_lasttimestamp):
            if 1:
                print('bbox {}, time={}, Pos = [{},{},{}]'.format(m.id,  m.header.stamp, tx,ty,tz))
                # bboxtime = m.header.stamp
                bboxes.append([tx, ty, tz, w, l, rz, h, rx, ry])

        return bboxes

    def ros_publish_final_bbox(self):
        markerArray = MarkerArray()
        for i in range(self.tracked_targets.shape[0]):
            tx, ty, tz, w, l, rz,  h, rx, ry = self.tracked_targets[i,:]
            m = Marker()
            m.type = Marker.CUBE
            m.header.frame_id = "velodyne"
            m.header.stamp = self.bbox_lasttimestamp
            m.scale.x, m.scale.y, m.scale.z = l, w, h
            m.pose.position.x, m.pose.position.y, m.pose.position.z = tx, ty, tz
            quaternion = tf.transformations.quaternion_from_euler(rx, ry, rz)
            m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = \
                quaternion[0],quaternion[1],quaternion[2],quaternion[3]
            m.color.a, m.color.r, m.color.g, m.color.b = \
                0.5, 0.0, 1.0, 0.0
            markerArray.markers.append(m)

        self.pub.publish(markerArray)

    def startlistening(self):
        rospy.init_node('tracker', anonymous=True)
        #rospy.Subscriber('/image_raw', Image, self.handle_image_msg) # for frame number
        rospy.Subscriber('/velodyne_points', PointCloud2, self.handle_lidar_msg) # for timing data
        rospy.Subscriber('/velodyne_points', PointCloud2, self.handle_lidar_msg) # for timing data
        rospy.Subscriber("/bbox", MarkerArray, self.handle_bbox_msg)
        self.pub = rospy.Publisher("bbox_final", MarkerArray, queue_size=1)
        print('tracker node initialzed')
        rospy.spin()

if __name__ == "__main__":
    tracker = Tracker()
    tracker.startlistening()

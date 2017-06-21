#!/usr/bin/env python

import rospy
import numpy as np
import tf
from visualization_msgs.msg import Marker,MarkerArray
from sensor_msgs.msg import PointCloud2, Image
from Tracklet_saver import *

class Tracker:

    def __init__(self):
        self.current_time = rospy.Time()
        self.tracklet_saver = Tracklet_saver('./')
        self.lidar_lasttimestamp = -1
        self.tracklet_lasttimestamp = -1
        self.tracklet_generated = False
        self.cameraframeindex = -1

    def write_final_tracklet_xml(self):
        if (not self.tracklet_generated):
            self.tracklet_saver.write_tracklet()
            self.tracklet_generated = True
            print '---------tracklet exported------------------'
            rospy.signal_shutdown('Tracklet exported')

    def save_gen_tracklet(self, frameid, scale, trans, rot): #, timestamp, bbox):
        #print 'Save tracklet'
        self.tracklet_saver.add_tracklet(frameid, scale, trans, rot)

    def kalman_update(self, frameid, trackid, scale, trans, rot):
        #print 'Kalman update #', trackid
        #########################################################
        #TODO: Kalman filter measurement update for single target
        #########################################################
        self.save_gen_tracklet(frameid, scale, trans, rot)

    def hungarian_update(self, markerarry):
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
                    print 'bbox ',m.id, ' time = ', m.header.stamp.to_nsec(), ': Pos = [',trans,']'
                    bboxtime = m.header.stamp.to_nsec()
                    trackindex += 1

                    #########################################################
                    #TODO: multi-target tracking algorithm update
                    #########################################################
                    self.kalman_update(self.cameraframeindex, trackindex, scale, trans, rot)

            self.tracklet_lasttimestamp = bboxtime

    def handle_image_msg(self, msg):
        self.cameraframeindex += 1

    def handle_lidar_msg(self, msg):
        if (not self.tracklet_generated):
            timestamplidar = msg.header.stamp.to_nsec()
            if timestamplidar < self.lidar_lasttimestamp: # rosplay loops
                print 'generating tracklet'
                self.write_final_tracklet_xml()
            else:
                self.lidar_lasttimestamp = timestamplidar
                print 'lidar time ', self.lidar_lasttimestamp

    def handle_bbox_msg(self,msg):
        if (not self.tracklet_generated) and (len(msg.markers)>0):
            self.tracklet_lasttimestamp = msg.markers[0].header.stamp.to_nsec()
            print 'bbox time  ', self.tracklet_lasttimestamp, ', Total bbox in this frame = ', len(msg.markers)
            m = msg.markers[0]
            print 'tx,ty,tz:', m.pose.position.x, m.pose.position.y, m.pose.position.z
            print 'w,l,h:', m.scale.x, m.scale.y, m.scale.z
            print 'rx,ry,rz:', m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z
            self.hungarian_update(msg.markers)


    def startlistening(self):
        rospy.init_node('tracker', anonymous=True)
        rospy.Subscriber('/image_raw', Image, self.handle_image_msg) # for frame number
        rospy.Subscriber('/velodyne_points', PointCloud2, self.handle_lidar_msg) # for timing data
        rospy.Subscriber("/bbox", MarkerArray, self.handle_bbox_msg)
        print 'tracker node initialzed'
        rospy.spin()

if __name__ == "__main__":
    tracker = Tracker()
    tracker.startlistening()

#!/usr/bin/env python

import rospy
import numpy as np
from visualization_msgs.msg import Marker,MarkerArray
from sensor_msgs.msg import Image

class Tracker:

    def __init__(self):
        self.current_time = rospy.Time()
        self.latestframetime = 0
        self.trans = [None]*3

    def save_gen_tracklet(self): #, timestamp, bbox):
        print 'Save and Generate tracklet.xml'
        #########################################################
        #TODO: save for tracklet generation at the end of rosplay
        #########################################################

    def kalman_update(self, trans, trackid):
        print 'Kalman update #', trackid
        #########################################################
        #TODO: Kalman filter measurement update for single target
        #########################################################
        self.save_gen_tracklet()

    def hungarian_update(self, translist):
        print 'Hungarian update'
        #########################################################
        #TODO: Hungarian algorithm update to separate targets
        #########################################################

        trackid = 0
        for trans in translist:
            self.kalman_update(trans, trackid)
            trackid += 1

    def handle_image_msg(self, msg):
        self.latestframetime = msg.header.stamp.to_nsec()
        print 'latest camera frame time = ', self.latestframetime

    def handle_bbox_msg(self,msg):
        print 'new bbox message'

        translist = []
        index = 0
        bboxtime = msg.markers[0].header.stamp
        for m in msg.markers:

            trans = m.pose.position

            tx = trans.x
            ty = trans.y
            tz = trans.z

            if  m.header.stamp == bboxtime:
                print 'bbox ',m.id, ' time = ', m.header.stamp, ': Pos = [',tx,',',ty,',',tz,']'
                translist.append(trans)
                bboxtime = m.header.stamp
                index += 1

        print 'Total bbox in this frame = ', index
        self.hungarian_update(translist)


    def startlistening(self):
        rospy.init_node('tracker', anonymous=True)
        rospy.Subscriber('/image_raw', Image, self.handle_image_msg)
        rospy.Subscriber("/bbox", MarkerArray, self.handle_bbox_msg)
        print 'tracker node initialzed'
        rospy.spin()

if __name__ == "__main__":
    tracker = Tracker()
    tracker.startlistening()

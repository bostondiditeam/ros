"""Multi object tracking using Kalman Filter.

This is a modification of:

SORT: A Simple, Online and Realtime Tracker
Copyright (C) 2016 Alex Bewley alex@dynamicdetection.com.

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
"""
from __future__ import print_function

from numba import jit
import os.path
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from skimage import io
from sklearn.utils.linear_assignment_ import linear_assignment
import glob
import time
import argparse
from filterpy.kalman import KalmanFilter

from collections import defaultdict, deque
from functools import partial
import warnings


@jit
def iou(bb_test_, bb_gt_):
    """
    Computes IUO between two bboxes in the form [x,y,w,h]

    """
    bb_test = convert_bbox_center_to_corners(
        bb_test_)  # convert to [x1,y1,w,h] to [x1,y1,x2,y2]
    bb_gt = convert_bbox_center_to_corners(bb_gt_)

    xx1 = np.maximum(bb_test[0], bb_gt[0])
    yy1 = np.maximum(bb_test[1], bb_gt[1])
    xx2 = np.minimum(bb_test[2], bb_gt[2])
    yy2 = np.minimum(bb_test[3], bb_gt[3])
    w = np.maximum(0., xx2 - xx1)
    h = np.maximum(0., yy2 - yy1)
    wh = w * h
    o = wh / ((bb_test[2] - bb_test[0]) * (bb_test[3] - bb_test[1])
              + (bb_gt[2] - bb_gt[0]) * (bb_gt[3] - bb_gt[1]) - wh)
    return(o)


@jit
def squared_diff(a, b):
    return (a - b) ** (2)


@jit
def euclidean(bb_test_, bb_gt_):
    """
    Computes similarity using euclidean distance between
    two bboxes in the form [x, y, s, r, yaw]
    using  1/ (1 + euclidean_dist)

    """
    x1, y1, s1, r1, yaw1 = get_bbox(bb_test_)
    x2, y2, s2, r2, yaw2 = get_bbox(bb_gt_)

    # o = (np.sum(squared_diff(i,j) for (i,j) in [(x1, x2), (y1, y2), (yaw1, yaw2)]))
    # this is not jit compatible. resort to using for loop:

    output = 0.
    for (i, j) in [(x1, x2), (y1, y2), (yaw1, yaw2), (s1, s2), (r1, r2)]:
        output += squared_diff(i, j)
    output = 1./(1. + (output ** (1 / 2.)))
    # print('distance {}'.format(o))
    return(output)


@jit
def distance(bb_test_, bb_gt_):
    # hard coded selection of method to compute similarity
    method = 'euclidean'
    if method == 'iou':
        # iou is currently NOT defined for bboxes in different orientations
        o = iou(bb_test_, bb_gt_)
    elif method == 'euclidean':
        o = euclidean(bb_test_, bb_gt_)
    return o


def convert_bbox_center_to_corners(bbox):
    """[x,y,h,w, yaw[,score]] --> [x1,y1, x2, y2"""
    # warnings.warn(str(len(bbox)))
    if len(bbox) > 5:
        x, y, h, w, yaw, score = bbox
    else:
        x, y, h, w, yaw = bbox
    return [x - w / 2., y - h / 2, x + w / 2., y + h / 2.]


@jit
def get_bbox(bbox):
    """Drop score from bbox (if any, last index)
    [x,y,h,w, yaw[,score]] --> [x1,y1, x2, y2"""
    # warnings.warn(str(len(bbox)))
    if len(bbox) > 5:
        x, y, h, w, yaw, score = bbox
    else:
        x, y, h, w, yaw = bbox
    return [x, y, h, w, yaw]


def convert_bbox_to_z(bbox):
    """
    Takes a bounding box in the form [x,y,w,h,yaw] and returns z in the form
      [x,y,s,r,yaw] where x,y is the centre of the box and s is the scale/area and r is
      the aspect ratio


      [x,y,w,h,yaw] -> [x,y,s,r,yaw]

    """
    w = bbox[2]
    h = bbox[3]
    x = bbox[0]
    y = bbox[1]
    s = w * h  # scale is just area
    r = w / float(h)
    # return np.array([x, y, s, r]).reshape((4, 1))
    yaw = bbox[4]
    return np.array([x, y, s, r, yaw]).reshape((5, 1))


def convert_x_to_bbox(x, score=None):
    """
    Takes a bounding box in the centre form [x,y,s,r, yaw] and returns it in the form
      [x, y, w, h, yaw] where x, y is the center
    """
    w = np.sqrt(x[2] * x[3])
    h = x[2] / w
    yaw = x[4]
    if(score is None):
        return np.array([x[0], x[1], w, h, yaw]).reshape((1, 5))
    else:
        return np.array([x[0], x[1], w, h, yaw, score]).reshape((1, 6))


class KalmanBoxTracker(object):
    """
    This class represents the internel state of individual tracked objects observed as bbox.
    """
    count = 0

    def __init__(self, bbox, attrs):
        """
        Initialises a tracker using initial bounding box.
        """
        # define constant velocity model
        # originalx : [u, v, s, r, |dot{u}, \dot{v}, \dot{s}]
        # adding \yaw, \dot{\yaw}
        # new x: [u, v, s, r, \yaw, |dot{u}, \dot{v}, \dot{s}, \dot{\yaw}]
        # assume r constant
        # dim_x : length of x vector
        # dim_z: numer of sensors measurements [x, y, s, r, yaw]
        self.kf = KalmanFilter(dim_x=9, dim_z=5)
        self.kf.F = np.array([[1, 0, 0, 0, 0, 1, 0, 0, 0],
                              [0, 1, 0, 0, 0, 0, 1, 0, 0],
                              [0, 0, 1, 0, 0, 0, 0, 1, 0],
                              [0, 0, 0, 1, 0, 0, 0, 0, 0],
                              [0, 0, 0, 0, 1, 0, 0, 0, 1],
                              [0, 0, 0, 0, 0, 1, 0, 0, 0],
                              [0, 0, 0, 0, 0, 0, 1, 0, 0],
                              [0, 0, 0, 0, 0, 0, 0, 1, 0],
                              [0, 0, 0, 0, 0, 0, 0, 0, 1]])

        # dim H: (dim_z, dim_x)
        self.kf.H = np.array([[1, 0, 0, 0, 0, 0, 0, 0, 0],
                              [0, 1, 0, 0, 0, 0, 0, 0, 0],
                              [0, 0, 1, 0, 0, 0, 0, 0, 0],
                              [0, 0, 0, 1, 0, 0, 0, 0, 0],
                              [0, 0, 0, 0, 1, 0, 0, 0, 0]])

        self.kf.R[2:, 2:] *= 10.
        self.kf.P[5:, 5:] *= 1000.  # give high uncertainty to the unobservable initial velocities
        self.kf.P *= 10.
        self.kf.Q[-1, -1] *= 0.01
        self.kf.Q[5:, 5:] *= 0.01

        self.kf.x[:5] = convert_bbox_to_z(bbox)
        self.time_since_update = 0
        self.id = KalmanBoxTracker.count
        KalmanBoxTracker.count += 1
        self.history = []
        self.hits = 0
        self.hit_streak = 0
        self.age = 0

        self.unused_box_attrs = attrs

    def update(self, bbox, attrs):
        """
        Updates the state vector with observed bbox.
        """
        self.time_since_update = 0
        self.history = []
        self.hits += 1
        self.hit_streak += 1
        self.kf.update(convert_bbox_to_z(bbox))

        self.unused_box_attrs = attrs

    def predict(self):
        """
        Advances the state vector and returns the predicted bounding box estimate.
        """
        if((self.kf.x[7] + self.kf.x[2]) <= 0):
            self.kf.x[7] *= 0.0
        self.kf.predict()
        self.age += 1
        if(self.time_since_update > 0):
            self.hit_streak = 0
        self.time_since_update += 1
        self.history.append(convert_x_to_bbox(self.kf.x))
        return self.history[-1]

    def get_state(self):
        """
        Returns the current bounding box estimate.
        """
        return convert_x_to_bbox(self.kf.x)


# @jit
def associate_detections_to_trackers(
        detections, trackers, distance_threshold=0.3):
    """
    Assigns detections to tracked object (both represented as bounding boxes)

    Returns 3 lists of matches, unmatched_detections and unmatched_trackers
    """
    if(len(trackers) == 0):
        return np.empty((0, 2), dtype=int), np.arange(
            len(detections)), np.empty((0, 5), dtype=int)
    distance_matrix = np.zeros(
        (len(detections), len(trackers)), dtype=np.float32)

    for d, det in enumerate(detections):
        for t, trk in enumerate(trackers):
            distance_matrix[d, t] = distance(det, trk)
            # print('distance of new det:{} to tracker {} = {}'.format(
            #     d, t, distance_matrix[d, t]))

    # warnings.warn(str(distance_matrix))
    # warnings.warn('tracking')

    matched_indices = linear_assignment(-distance_matrix)

    unmatched_detections = []
    for d, det in enumerate(detections):
        if(d not in matched_indices[:, 0]):
            unmatched_detections.append(d)
    unmatched_trackers = []
    for t, trk in enumerate(trackers):
        if(t not in matched_indices[:, 1]):
            unmatched_trackers.append(t)

    # filter out matched with low distance
    matches = []
    for m in matched_indices:
        if(distance_matrix[m[0], m[1]] < distance_threshold):
            unmatched_detections.append(m[0])
            unmatched_trackers.append(m[1])
        else:
            matches.append(m.reshape(1, 2))
    if(len(matches) == 0):
        matches = np.empty((0, 2), dtype=int)
    else:
        matches = np.concatenate(matches, axis=0)

    return matches, np.array(
        unmatched_detections), np.array(unmatched_trackers)


class Sort(object):
    def __init__(self, max_age=1, min_hits=3, distance_threshold=.3):
        """
        Sets key parameters for SORT
        """
        self.max_age = max_age
        self.min_hits = min_hits
        self.trackers = []
        self.frame_count = 0
        self.distance_threshold = distance_threshold

    def update(self, detections):
        """
        Params:
            dets (:obj:`numpy.array`) : a numpy array of detections
                in the format [[x,y,w,h, yaw, score],
                                [x,y,w,h, yaw,score],...]

        Requires: this method must be called once for each frame even with empty detections.
        Returns the a similar array, where the last column is the object ID.

        NOTE: The number of objects returned may differ from the number of detections provided.
        """
        self.frame_count += 1
        # get predicted locations from existing trackers.
        trks = np.zeros((len(self.trackers), 6))

        to_del = []
        ret = []
        ret_attrs = []
        # print('trks', trks)
        for t, trk in enumerate(trks):
            pos = self.trackers[t].predict()[0]
            trk[:] = [pos[0], pos[1], pos[2], pos[3], pos[4], 0]
            if(np.any(np.isnan(pos))):
                to_del.append(t)
                # warnings.warn('popping trackers {}'.format(t))

        trks = np.ma.compress_rows(np.ma.masked_invalid(trks))
        for t in reversed(to_del):
            self.trackers.pop(t)
            # warnings.warn('popping trackers {}'.format(t))

        # separate unused attributes
        dets, attrs = self.split_detections(detections)
        # print('attrs',attrs.shape)

        # print(dets.shape, trks.shape)
        # warnings.warn('before computing trackers')
        matched, unmatched_dets, unmatched_trks = associate_detections_to_trackers(
            dets, trks, distance_threshold=self.distance_threshold)
        # print(matched, unmatched_dets, unmatched_trks)

        # update matched trackers with assigned detections
        for t, trk in enumerate(self.trackers):
            if(t not in unmatched_trks):
                d = matched[np.where(matched[:, 1] == t)[0], 0]
                trk.update(dets[d, :][0], attrs[d, :][0])
                # print('printing attrs')
                # print(attrs[d, :][0])
                # print('*')
                # print(trk.unused_box_attrs)

        # create and initialise new trackers for unmatched detections
        # print(dets.shape)
        for i in unmatched_dets:
            trk = KalmanBoxTracker(dets[i, :5], attrs[i, :])
            self.trackers.append(trk)
        i = len(self.trackers)
        for trk in reversed(self.trackers):
            d = trk.get_state()[0]
            d_attr = trk.unused_box_attrs
            # print('printing d_attr')
            # print(d_attr)
            if((trk.time_since_update < 1) and (trk.hit_streak >= self.min_hits or self.frame_count <= self.min_hits)):
                # +1 as MOT benchmark requires positive
                ret.append(np.concatenate((d, [trk.id + 1])).reshape(1, -1))
                ret_attrs.append(d_attr)
            i -= 1
            # remove dead tracklet
            if(trk.time_since_update > self.max_age):
                self.trackers.pop(i)
        if(len(ret) > 0):
            # return np.concatenate(ret)
            # last column is track id
            tracked_detections = np.concatenate(ret)[:,:-1]
            tracked_attrs = np.array(ret_attrs)
            # print('output shape')
            # print(tracked_detections.shape, tracked_attrs.shape)
            # print(np.concatenate((tracked_detections,tracked_attrs),axis=1).shape)
            return np.concatenate((tracked_detections, tracked_attrs),axis=1), np.concatenate(ret)[:,-1]
        return np.empty((0, 5)), np.empty((0, 1))

    def split_detections(self, detections):
        dets = detections[:, :5]
        dets = np.insert(dets, 5, 0., axis=1)
        attrs = detections[:, 5:]
        return dets, attrs


if __name__ == '__main__':
    pass

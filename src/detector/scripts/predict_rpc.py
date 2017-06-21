from twisted.web import xmlrpc, server
import numpy as np

import os
import numpy as np
import cv2
import sys
import time

sys.path.append(os.path.join(sys.path[0],"../MV3D/src"))

import mv3d
import math
from net.processing.boxes3d import boxes3d_decompose
from data import Preprocess

print("init the network")
LIDAR_TOP_SHAPE = (500, 300, 15) # to be decided (400, 400, 8)
LIDAR_FRONT_SHAPE = (0,)  # to be decided.
RGB_SHAPE = (596, 1368, 3)  # to be decided (1096, 1368, 3)

#m3 = mod.MV3D()
#m3.predict_init(LIDAR_TOP_SHAPE, LIDAR_FRONT_SHAPE, RGB_SHAPE)
process = Preprocess()
predict = mv3d.Predictor(LIDAR_TOP_SHAPE, LIDAR_FRONT_SHAPE, RGB_SHAPE, log_tag='default-didi2')

print("testing predict:")
dir = os.path.join(sys.path[0], "/ext2/round2_data/output/suburu_driving_past_it/suburu07")
rgb_path = os.path.join(dir, "image_02/data", "1492888603962510690.png")
rgb = process.rgb(cv2.imread(rgb_path))
print(rgb.shape)
lidar_path = os.path.join(dir, "velodyne_points/data", "1492888603962510690.bin")
lidar =np.fromfile(lidar_path, np.float32)
lidar = lidar.reshape((-1, 4))
top = process.lidar_to_top(lidar)
front = np.zeros((0, ), dtype=np.float32)

np_reshape = lambda np_array: np_array.reshape(1, *(np_array.shape))
top = np_reshape(top)
front = np_reshape(front)
rgb = np_reshape(rgb)

start = time.time()
boxes3d, probs = predict(top, front, rgb)
translation, size, rotation = boxes3d_decompose(boxes3d)
timestamp = time.time()-start
print("predict boxes len=%d use %f seconds" % (len(boxes3d),timestamp))
print("test translation: {}".format(translation))
print("test size: {}".format(size))
print("test rotation: {}".format(rotation))

class MioServer(xmlrpc.XMLRPC):
    """
    An example object to be published.
    """
    def xmlrpc_test0(self):
        return np.sqrt(2).tolist()

    def xmlrpc_test1(self):
        print("call xmlrpc_test1")
        return [[1, -1], [2, -2], [3, -3]] #np.sqrt(2)

    def xmlrpc_test2(self):
        print("call xmlrpc_test2")
        a = np.array([[1., -1.], [2., -2.], [3., -3.]])
        return a.tolist();

    def xmlrpc_predict(self):
        top_reshaped = np.load(os.path.join(sys.path[0], "../MV3D/data/", "top.npy"))
        rgb_reshaped = np.load(os.path.join(sys.path[0], "../MV3D/data/", "rgb.npy"))
        print("receice rpc pridect request: top-{} rgb-{}".format(top_reshaped.shape, rgb_reshaped.shape))
        start = time.time()
        boxes3d, probs = predict(top_reshaped, front, rgb_reshaped)
        print("predict boxes3d:{} use time {}".format(boxes3d.shape, time.time()-start))
        return boxes3d.tolist()

if __name__ == '__main__':
    from twisted.internet import reactor

    print("begin receive request")
    r = MioServer()
    reactor.listenTCP(8080, server.Site(r))
    reactor.run()
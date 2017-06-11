from twisted.web import xmlrpc, server
import numpy as np

import os
import numpy as np
import cv2
import sys
import time

sys.path.append(os.path.join(sys.path[0],"../MV3D/src"))

import model as mod
from data import draw_top_image,draw_box3d_on_top
from net.utility.draw import imsave ,draw_box3d_on_camera, draw_box3d_on_camera
from net.processing.boxes3d import boxes3d_decompose

print("init the network")
LIDAR_TOP_SHAPE = (450, 100, 9) # to be decided (400, 400, 8)
LIDAR_FRONT_SHAPE = (1, 1)  # to be decided.
RGB_SHAPE = (596, 1368, 3)  # to be decided (1096, 1368, 3)

m3 = mod.MV3D()
m3.predict_init(LIDAR_TOP_SHAPE, LIDAR_FRONT_SHAPE, RGB_SHAPE)

print("testing predict:")
dir = os.path.join(sys.path[0], "../MV3D/data/preprocessed/didi")

rgb_path = os.path.join(dir, "rgb", "1/6_f", "00000.png")
rgb = cv2.imread(rgb_path)
top_path = os.path.join(dir, "top", "1/6_f", "00000.npy")
top = np.load(top_path)
front = np.zeros((1, 1), dtype=np.float32)

np_reshape = lambda np_array: np_array.reshape(1, *(np_array.shape))
top = np_reshape(top)
front = np_reshape(front)
rgb = np_reshape(rgb)

boxes3d, probs = m3.predict(top, front, rgb)
print("predict boxes len=%d" % len(boxes3d))

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
        boxes3d, probs = m3.predict(top_reshaped, front, rgb_reshaped)
        print("predict boxes3d:{} use time {}".format(boxes3d.shape, time.time()-start))
        return boxes3d.tolist()

if __name__ == '__main__':
    from twisted.internet import reactor

    print("begin receive request")
    r = MioServer()
    reactor.listenTCP(8080, server.Site(r))
    reactor.run()
import numpy as np

# a = np.array([1.,2.,3.,4.,5.])
# b = np.array([[1.,1.,1.,1.,1.],
#              [2.,2.,2.,2.,2.],
#              [3.,3.,3.,3.,3.]])
# c = np.array([[[-3., -1], [-3., -1.], [-3., -1.], [-3., -1], [-3., -1]],
#              [[-2., -2], [-2., -2.], [-2., -2.], [-2., -2], [-2., -2]],
#              [[-1., -3], [-1., -3.], [-1., -3.], [-1., -3], [-1., -3]]])

# print(a.shape)
# print(b.shape)
# print(c.shape)
# print("============result====================")
print("xxxxxxxxxxxxxxxxxxxxxxxxxxxxx")


def func(_a, _b, _c):
    print(_a.shape)
    print(_b.shape)
    print(_c.shape)
    print("=============func================")
    x = _b * _a.T
    _d = np.zeros(_c.shape)
    _d[:, :, 0] = _b
    _d[:, :, 1] = -_b
    y = _c ** _d
    print("y.shape : ")
    print(y.shape)
    print(y)
    return _c ** _d

# d = func(a, b, c)
# print(d)

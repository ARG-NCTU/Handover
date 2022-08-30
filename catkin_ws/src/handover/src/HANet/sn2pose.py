#!/usr/bin/env python3

import os
import numpy as np
from math import atan, tan, pi
from scipy.spatial.transform import Rotation
from math import *
import matplotlib.pyplot as plt
import python.lib.python.NormalEstimatorHough as Estimator
# from python.lib.python.NormalEstimatorHough import Estimator

class SurfaceNormal2Quaternion():
    def __init__(self, fx, fy, cx, cy):
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy
        self.wild = 20

    def get_quaternion(self, depth, pos_x, pos_y, theta, ratio=1e-3):
        H, W = depth.shape
        self.fov_x = 2 * atan(W / (2 * self.fx))
        self.fov_y = 2 * atan(H / (2 * self.fy))
        point_cloud = []

        # change value to meters
        img = depth.astype('float64')
        img *= ratio

        for i in range(pos_x - self.wild, pos_x + self.wild + 1, 1):
            for j in range(pos_y - self.wild, pos_y + self.wild + 1, 1):
                alpha = (pi - self.fov_x) / 2
                gamma = alpha + self.fov_x * float((W - j) / W)
                delta_x = img[i, j] / tan(gamma)

                alpha = (pi - self.fov_y) / 2
                gamma = alpha + self.fov_y * float((H - i) / H)
                delta_y = img[i, j] / tan(gamma)

                point_cloud.append([delta_x, delta_y, float(img[i, j])])

        estimator = Estimator.NormalEstimatorHough()
        estimator.set_points(np.array(point_cloud))
        estimator.set_K(50)
        estimator.estimate_normals()
        tmp = estimator.get_normals()[int((2*self.wild + 1)/2)]
        tmp2 = -1*np.array(tmp)
        rotations_vector = np.array([tmp2[2], tmp2[1], tmp2[0]])
        # print(rotations_vector)
        r = Rotation.from_rotvec(theta * rotations_vector, degrees=True)
        q = r.as_quat()
        # print(q)

        return q

if __name__ == '__main__':
    a = 387.59381103515625
    b = 387.59381103515625
    c = 321.61895751953125
    d = 239.87408447265625

    depth_img = np.zeros((224,224))
    depth_img += 15000
    depth_img[50:200, 50:200] = 2000

    transformer = SurfaceNormal2Quaternion(a,b,c,d)

    Q = transformer.get_quaternion(depth_img, 112, 112, 90)
    print(Q)

    Q2 = transformer.get_quaternion(depth_img, 112, 112, 0)
    print(Q2)

    # plt.imshow(depth_img)
    # plt.show()
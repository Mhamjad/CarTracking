import numpy as np
import cv2

K = np.array([[1557.24, 0., 944],
              [0., 1486.06, 800.49],
              [0., 0., 1.]])
D = np.array([-0.264878285772124, 0.065888395493354, -0.00199645197136479, 0])

K_300 = np.array([[182.4890625, 0.0, 110.625],
                 [0.0, 309.59583333, 166.76875],
                 [0.0, 0.0, 1.0]])

def undistort(frame):
    return cv2.undistort(frame, K_300, D)

def undistort_2560(frame):
    return cv2.undistort(frame, K, D)

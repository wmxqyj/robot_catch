import matplotlib.pyplot as plt
import numpy as np
import time
import cv2
from scipy import optimize

from mpl_toolkits.mplot3d import Axes3D
from camera.orbbec import get_camera_data

while True:
    checkerboard_size = (4, 4)
    refine_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    camera_color_img, camera_depth_img = get_camera_data()     # TODO   获取相机的color和depth图
    gray_data = cv2.cvtColor(camera_color_img, cv2.COLOR_BGR2GRAY)
    checkerboard_found, corners = cv2.findChessboardCorners(gray_data, checkerboard_size, None, cv2.CALIB_CB_ADAPTIVE_THRESH)
    print(checkerboard_found)

    # 如果检测到标定板
    if checkerboard_found:
        corners_refined = cv2.cornerSubPix(gray_data, corners, (4, 4), (-1, -1), refine_criteria)

        # Get observed checkerboard center 3D point in camera space
        checkerboard_pix = np.round(corners_refined[5, 0, :]).astype(int)
        checkerboard_z = camera_depth_img[checkerboard_pix[1]][checkerboard_pix[0]]

        # Draw and display the corners
        vis = cv2.drawChessboardCorners(camera_color_img, (1, 1), corners_refined[5, :, :], checkerboard_found)
        cv2.imwrite('%06d.png' , vis)
        cv2.imshow('Calibration', vis)
        cv2.waitKey(1)

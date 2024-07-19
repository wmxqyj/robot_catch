import matplotlib.pyplot as plt
import numpy as np
import time
import cv2
from scipy import optimize

from mpl_toolkits.mplot3d import Axes3D
from camera.orbbec import get_camera_data

checkerboard_size = (4, 4)
refine_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
camera_color_img, camera_depth_img = get_camera_data()     # TODO   获取相机的color和depth图
bgr_color_data = cv2.cvtColor(camera_color_img, cv2.COLOR_RGB2BGR)
gray_data = cv2.cvtColor(bgr_color_data, cv2.COLOR_RGB2GRAY)
checkerboard_found, corners = cv2.findChessboardCorners(gray_data, checkerboard_size, None, cv2.CALIB_CB_ADAPTIVE_THRESH)
print(checkerboard_found)

# if checkerboard_found:
#     corners_refined = cv2.cornerSubPix(gray_data, corners, (5, 5), (-1, -1), refine_criteria)
#
#     # Get observed checkerboard center 3D point in camera space
#     checkerboard_pix = np.round(corners_refined[12, 0, :]).astype(int)
#     checkerboard_z = camera_depth_img[checkerboard_pix[1]][checkerboard_pix[0]]
#     checkerboard_x = np.multiply(checkerboard_pix[0] - robot.cam_intrinsics[0][2],
#                                  checkerboard_z / robot.cam_intrinsics[0][0])
#     checkerboard_y = np.multiply(checkerboard_pix[1] - robot.cam_intrinsics[1][2],
#                                  checkerboard_z / robot.cam_intrinsics[1][1])
#     if checkerboard_z == 0:
#         continue
#
#     # Save calibration point and observed checkerboard center
#     observed_pts.append([checkerboard_x, checkerboard_y, checkerboard_z])
#     # tool_position[2] += checkerboard_offset_from_tool
#     tool_position = tool_position + checkerboard_offset_from_tool
#
#     measured_pts.append(tool_position)
#     observed_pix.append(checkerboard_pix)
#
#     # Draw and display the corners
#     # vis = cv2.drawChessboardCorners(robot.camera.color_data, checkerboard_size, corners_refined, checkerboard_found)
#     vis = cv2.drawChessboardCorners(bgr_color_data, (1, 1), corners_refined[12, :, :], checkerboard_found)
#     cv2.imwrite('%06d.png' % len(measured_pts), vis)
#     cv2.imshow('Calibration', vis)
#     cv2.waitKey(1000)
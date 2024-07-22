#!/usr/bin/env python
# -*- coding:utf-8 -*-
# author:qyj
# datetime:2024/7/21 21:32
# software: PyCharm

import matplotlib.pyplot as plt
import numpy as np
import time
import cv2
from scipy import optimize
from camera.orbbec import *
from mpl_toolkits.mplot3d import Axes3D

# Cols: min max, Rows: x y z (define workspace limits in robot coordinates)
workspace_limits = np.asarray([[0.2, 0.4], [0.4, 0.6], [0.05, 0.1]])         # 机器人工具坐标系
calib_grid_step = 0.05                            # 0.05m, step_size,there are 50 points
checkerboard_offset_from_tool = [0, 0.121, 0]
tool_orientation = [-np.pi/2, 0, 0]               # gripper orientation

# Construct 3D calibration grid across workspace
print(1 + (workspace_limits[0][1] - workspace_limits[0][0])/calib_grid_step)
gridspace_x = np.linspace(workspace_limits[0][0], workspace_limits[0][1], int(1 + (workspace_limits[0][1] - workspace_limits[0][0])/calib_grid_step))
gridspace_y = np.linspace(workspace_limits[1][0], workspace_limits[1][1], int(1 + (workspace_limits[1][1] - workspace_limits[1][0])/calib_grid_step))
gridspace_z = np.linspace(workspace_limits[2][0], workspace_limits[2][1], int(1 + (workspace_limits[2][1] - workspace_limits[2][0])/calib_grid_step))
calib_grid_x, calib_grid_y, calib_grid_z = np.meshgrid(gridspace_x, gridspace_y, gridspace_z)   # calib_grid_xx中的每个元素表示对应位置的x的坐标
num_calib_grid_pts = calib_grid_x.shape[0]*calib_grid_x.shape[1]*calib_grid_x.shape[2]              # 返回三维网格中的总点数，共125个

calib_grid_x.shape = (num_calib_grid_pts,1)
calib_grid_y.shape = (num_calib_grid_pts,1)
calib_grid_z.shape = (num_calib_grid_pts,1)                                                         # 重塑数组转换由（5,5,5)的三维数组转化为(125,1)的二维数组
calib_grid_pts = np.concatenate((calib_grid_x, calib_grid_y, calib_grid_z), axis=1)           # 从第一列开始拼接，生成(x,y,z)类型的三维坐标

measured_pts = []         # 测量点
observed_pts = []         # 观测点
observed_pix = []         # 观测到的像素值

# Move robot to home pose


# Slow down robot

# Make robot gripper point upwards

# Move robot to each calibration point in workspace
print('Collecting data...')
for calib_pt_idx in range(num_calib_grid_pts):                                     # 共有多少点，全部抽取出来依次排列
    tool_position = calib_grid_pts[calib_pt_idx,:]                                 # 取出每一行的数据，即一个表示三维坐标的点
    tool_config = [tool_position[0],tool_position[1],tool_position[2],
           tool_orientation[0],tool_orientation[1],tool_orientation[2]]            # 形成完正的工具配置，表示工具的全部信息，包含(x,y,z,roll,pitch,yaw)
    tool_config1 = [tool_position[0], tool_position[1], tool_position[2],
                   tool_orientation[0], tool_orientation[1], tool_orientation[2]]
    print(f"tool position and orientation:{tool_config1}")
    # robot.move_j_p(tool_config)                                                    # TODO 机器人操作，替换为接口
    time.sleep(2)                                                                  # 停留2s，等待机械臂到达指定位置

    # Find checkerboard center
    checkerboard_size = (4, 4)                                            # 相机内角点数，方块数-1
    refine_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)      # 优化角点检测的准确性
    camera_color_img, camera_depth_img = get_camera_data()                     # TODO   获取相机的color和depth图
    gray_data = cv2.cvtColor(camera_color_img, cv2.COLOR_BGR2GRAY)
    checkerboard_found, corners = cv2.findChessboardCorners(gray_data, checkerboard_size, None, cv2.CALIB_CB_ADAPTIVE_THRESH)
    print(checkerboard_found)

    if checkerboard_found:         # 如果成功检测到标定板
        corners_refined = cv2.cornerSubPix(gray_data, corners, (4, 4), (-1, -1), refine_criteria)

        # Get observed checkerboard center 3D point in camera space
        checkerboard_pix = np.round(corners_refined[5, 0, :]).astype(int)               # 获取第6个角点坐标，包含(x,y)信息
        checkerboard_z = camera_depth_img[checkerboard_pix[1]][checkerboard_pix[0]]
        checkerboard_x = np.multiply(checkerboard_pix[0] - cam_intrinsics[0][2],   # TODO  相机内参矩阵，包含(fx,fy,cx,cy)，并计算棋盘在相机坐标系下的坐标
                                     checkerboard_z / cam_intrinsics[0][0])
        checkerboard_y = np.multiply(checkerboard_pix[1] - cam_intrinsics[1][2],
                                     checkerboard_z / cam_intrinsics[1][1])
        if checkerboard_z == 0:
            continue

        # Save calibration point and observed checkerboard center
        observed_pts.append([checkerboard_x, checkerboard_y, checkerboard_z])   # 将得到的数据储存到观测矩阵
        # tool_position[2] += checkerboard_offset_from_tool
        tool_position = tool_position + checkerboard_offset_from_tool

        measured_pts.append(tool_position)             # 将测量值添加到测量矩阵，即在观测矩阵上加上工具偏移量
        observed_pix.append(checkerboard_pix)          # 储存中心点的像素坐标到像素矩阵

        # Draw and display the corners
        vis = cv2.drawChessboardCorners(camera_color_img, (1, 1), corners_refined[5, :, :], checkerboard_found)
        cv2.imwrite('Log/%06d.png' % len(measured_pts), vis)
        cv2.imshow('Calibration', vis)
        cv2.waitKey(1000)


# Move robot back to home pose
# robot.go_home()

measured_pts = np.asarray(measured_pts)
observed_pts = np.asarray(observed_pts)
observed_pix = np.asarray(observed_pix)
world2camera = np.eye(4)              # 初始化一个世界坐标系到相机坐标系的旋转矩阵


# Estimate rigid transform with SVD (from Nghia Ho)
def get_rigid_transform(A, B):
    assert len(A) == len(B)
    N = A.shape[0] # Total points
    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)
    AA = A - np.tile(centroid_A, (N, 1)) # Centre the points
    BB = B - np.tile(centroid_B, (N, 1))
    H = np.dot(np.transpose(AA), BB) # Dot is matrix multiplication for array
    U, S, Vt = np.linalg.svd(H)
    R = np.dot(Vt.T, U.T)
    if np.linalg.det(R) < 0: # Special reflection case
       Vt[2,:] *= -1
       R = np.dot(Vt.T, U.T)
    t = np.dot(-R, centroid_A.T) + centroid_B.T
    return R, t


def get_rigid_transform_error(z_scale):
    global measured_pts, observed_pts, observed_pix, world2camera, camera

    # Apply z offset and compute new observed points using camera intrinsics
    observed_z = observed_pts[:,2:] * z_scale
    observed_x = np.multiply(observed_pix[:,[0]]-cam_intrinsics[0][2],observed_z/cam_intrinsics[0][0])
    observed_y = np.multiply(observed_pix[:,[1]]-cam_intrinsics[1][2],observed_z/cam_intrinsics[1][1])
    new_observed_pts = np.concatenate((observed_x, observed_y, observed_z), axis=1)

    # Estimate rigid transform between measured points and new observed points
    R, t = get_rigid_transform(np.asarray(measured_pts), np.asarray(new_observed_pts))
    t.shape = (3,1)
    world2camera = np.concatenate((np.concatenate((R, t), axis=1),np.array([[0, 0, 0, 1]])), axis=0)

    # Compute rigid transform error
    registered_pts = np.dot(R,np.transpose(measured_pts)) + np.tile(t,(1, measured_pts.shape[0]))
    error = np.transpose(registered_pts) - new_observed_pts
    error = np.sum(np.multiply(error,error))
    rmse = np.sqrt(error/measured_pts.shape[0])
    return rmse


# Optimize z scale w.r.t. rigid transform error.
print('Calibrating...')
z_scale_init = 1
optim_result = optimize.minimize(get_rigid_transform_error, np.asarray(z_scale_init), method='Nelder-Mead')
camera_depth_offset = optim_result.x

# Save camera optimized offset and camera pose
print('Saving...')
np.savetxt('camera_depth_scale.txt', camera_depth_offset, delimiter=' ')
get_rigid_transform_error(camera_depth_offset)
camera_pose = np.linalg.inv(world2camera)
np.savetxt('camera_pose.txt', camera_pose, delimiter=' ')
print('Done.')

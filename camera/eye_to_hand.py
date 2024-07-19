import matplotlib.pyplot as plt
import numpy as np
import time
import cv2
from scipy import optimize

from mpl_toolkits.mplot3d import Axes3D

# Cols: min max, Rows: x y z (define workspace limits in robot coordinates)
workspace_limits = np.asarray([[0.2, 0.4], [0.4, 0.6], [0.05, 0.1]])
calib_grid_step = 0.05                            # 0.05m, step_size,there are 50 points
checkerboard_offset_from_tool = [0, 0.121, 0]
tool_orientation = [-np.pi/2, 0, 0]               # gripper orientation

# Construct 3D calibration grid across workspace


# Move robot to home pose


# Slow down robot

# Make robot gripper point upwards

# Move robot to each calibration point in workspace

# Find checkerboard center
checkerboard_size = (4, 4)
refine_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
camera_color_img, camera_depth_img = robot.get_camera_data()
bgr_color_data = cv2.cvtColor(camera_color_img, cv2.COLOR_RGB2BGR)
gray_data = cv2.cvtColor(bgr_color_data, cv2.COLOR_RGB2GRAY)
checkerboard_found, corners = cv2.findChessboardCorners(gray_data, checkerboard_size, None, cv2.CALIB_CB_ADAPTIVE_THRESH)
print(checkerboard_found)



# Move robot back to home pose
# robot.go_home()


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
    observed_x = np.multiply(observed_pix[:,[0]]-robot.cam_intrinsics[0][2],observed_z/robot.cam_intrinsics[0][0])
    observed_y = np.multiply(observed_pix[:,[1]]-robot.cam_intrinsics[1][2],observed_z/robot.cam_intrinsics[1][1])
    new_observed_pts = np.concatenate((observed_x, observed_y, observed_z), axis=1)

    # Estimate rigid transform between measured points and new observed points
    R, t = get_rigid_transform(np.asarray(measured_pts), np.asarray(new_observed_pts))
    t.shape = (3,1)
    world2camera = np.concatenate((np.concatenate((R, t), axis=1),np.array([[0, 0, 0, 1]])), axis=0)

    # Compute rigid transform error
    registered_pts = np.dot(R,np.transpose(measured_pts)) + np.tile(t,(1,measured_pts.shape[0]))
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

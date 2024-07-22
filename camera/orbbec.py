import os

import cv2
import numpy as np

from pyorbbecsdk import *
from utils import *


def save_depth_frame(frame: DepthFrame, index):
    if frame is None:
        return
    width = frame.get_width()
    height = frame.get_height()
    timestamp = frame.get_timestamp()
    scale = frame.get_depth_scale()
    data = np.frombuffer(frame.get_data(), dtype=np.uint16)
    data = data.reshape((height, width))
    data = data.astype(np.float32) * scale
    data_fl32 = np.expand_dims(data, axis=2)
    data = data.astype(np.uint16)
    save_image_dir = os.path.join(os.getcwd(), "depth_images")
    if not os.path.exists(save_image_dir):
        os.mkdir(save_image_dir)
    raw_filename = save_image_dir + "/depth_{}x{}_{}_{}.raw".format(width, height, index, timestamp)
    data.tofile(raw_filename)
    return data_fl32, raw_filename


def save_color_frame(frame: ColorFrame, index):
    if frame is None:
        return
    width = frame.get_width()
    height = frame.get_height()
    timestamp = frame.get_timestamp()
    save_image_dir = os.path.join(os.getcwd(), "color_images")
    if not os.path.exists(save_image_dir):
        os.mkdir(save_image_dir)
    filename = save_image_dir + "/color_{}x{}_{}_{}.png".format(width, height, index, timestamp)
    data, image = frame_to_bgr_image(frame)
    if image is None:
        print("failed to convert frame to image")
        return
    cv2.imwrite(filename, image)
    return data, image


# get camera data,color and depth data
"""
返回的格式：
    color_image 3通道BGR图
    depth_image = np.asanyarray(aligned_depth_frame.get_data(), dtype=np.float32)
    depth_image = np.expand_dims(depth_image, axis=2)
"""
def get_camera_data():
    pipeline = Pipeline()
    config = Config()
    saved_color_cnt: int = 0
    saved_depth_cnt: int = 0
    has_color_sensor = False
    try:
        profile_list = pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
        if profile_list is not None:
            color_profile: VideoStreamProfile = profile_list.get_default_video_stream_profile()
            config.enable_stream(color_profile)
            has_color_sensor = True
    except OBError as e:
        print(e)
    depth_profile_list = pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)
    if depth_profile_list is not None:
        depth_profile = depth_profile_list.get_default_video_stream_profile()
        config.enable_stream(depth_profile)
    pipeline.start(config)
    while True:
        try:
            frames = pipeline.wait_for_frames(100)
            if frames is None:
                continue
            if has_color_sensor:
                if saved_color_cnt >= 1 and saved_depth_cnt >= 1:
                    break
            elif saved_depth_cnt >= 1:
                break
            color_frame = frames.get_color_frame()
            if color_frame is not None and saved_color_cnt < 1:
                color_img_data,  color_img= save_color_frame(color_frame, saved_color_cnt)
                saved_color_cnt += 1
            depth_frame = frames.get_depth_frame()
            if depth_frame is not None and saved_depth_cnt < 1:
                depth_img_data, depth_img = save_depth_frame(depth_frame, saved_depth_cnt)
                saved_depth_cnt += 1
        except KeyboardInterrupt:
            break
    return color_img, depth_img_data


"""
相机内参：
    格式： [[fx,0,ppx],
           [0,fy,ppy],
           [0,0,1]]
           可以通过标定获得：
           mtx:
 [[439.08239835   0.         321.42224963]
 [  0.         439.22251104 244.41306463]
 [  0.           0.           1.        ]]
"""
cam_intrinsics =[[439.08239835, 0, 321.42224963],
              [0, 439.22251104, 244.41306463],
              [0, 0, 1]]
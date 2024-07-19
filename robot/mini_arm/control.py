#!/usr/bin/env python 
# -*- coding:utf-8 -*-
# author:qyj 
# datetime:2024/7/19 20:14 
# software: PyCharm

import requests
import time
from fourAxis_attitude_solution import *

print("机械臂进行连接")
# 请求格式 http://192.168.137.46/command?X0;X180;X90&time=1720083413857


def sendData(arg):
    seconds_timestamp = time.time()                              # 当前秒级时间戳
    milliseconds_timestamp = int(seconds_timestamp * 1000)       # 转换为毫秒级时间戳

    url = f"http://192.168.137.32/command?{arg}&time={milliseconds_timestamp}"
    rq = requests.get(url)  # 发起GET请求,并用params传入设定参数
    print(rq.text)  # 将内容转为text格式


if __name__ == '__main__':
    # 定义笛卡尔坐标系下的目标坐标
    x = 0
    y = 10
    z = 5
    valid, deg1, deg2, deg3, deg4 = backward_kinematics(x, y, z, alpha=180)    # 逆运动学解算出关节坐标

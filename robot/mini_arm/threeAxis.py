#!/usr/bin/env python 
# -*- coding:utf-8 -*-
# author:qyj 
# datetime:2024/7/19 20:52 
# software: PyCharm

import math
from control import *


def calculate(x, y, z):
    L1 = 0.06  # 杆长
    L2 = 0.1
    L3 = 0.1
    pi = math.pi

    Bx = x - L3  # 这里设置γ为-90
    By = y
    lp = Bx ** 2 + By ** 2

    if math.sqrt(lp) >= L1 + L2 or math.sqrt(lp) <= abs(L1 - L2):
        return None  # 返回 None 表示输入不合法

    alpha = math.atan2(By, Bx)
    beta = math.acos((L1 ** 2 + lp - L2 ** 2) / (2 * L1 * math.sqrt(lp)))  # 这里使用弧度制

    ptheta0 = -(pi / 2.0 - alpha - beta)
    ptheta1 = math.acos((L1 ** 2 + L2 ** 2 - lp) / (2 * L1 * L2)) - pi
    ptheta2 = -ptheta0 - ptheta1 - pi / 2.0

    # 将弧度制转换为角度制
    ptheta0_deg = math.degrees(ptheta0)
    ptheta1_deg = math.degrees(ptheta1)
    ptheta2_deg = math.degrees(ptheta2)

    return ptheta0_deg, ptheta1_deg, ptheta2_deg


# 示例使用
x, y, z = 0.07, 0.0, 0.0
angles = calculate(x, y, z)

if angles is not None:
    print(f'计算出的角度为：theta0 = {angles[0]}, theta1 = {angles[1]}, theta2 = {angles[2]}')
    axisData = f"X{angles[0]};Y{angles[1]};Z{angles[2]}"
    # sendData(axisData)
else:
    print('输入的坐标不合法。')




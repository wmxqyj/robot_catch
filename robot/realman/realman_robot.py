from utils.robotic_arm import *

import sys

robot = Arm(RM65, '192.168.1.18')

print(robot.API_Version())



robot.Arm_Socket_Close()          # 关闭连接

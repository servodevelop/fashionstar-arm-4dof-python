'''
设置机械臂关节的弧度
* 作者: 阿凯
* Email: xingshunkai@qq.com
* 更新时间: 2020-02-18
'''
# 添加路径
import sys
sys.path.append('../src/')
# 导入依赖
import time
import math
from math import pi
from fs_arm_4dof import Arm4DoF
from config import *

# 机械臂初始化
arm = Arm4DoF()
# 设置关节的弧度
# - 关节1 -->   0
# - 关节2 --> -pi/2
# - 关节3 -->  pi/2
# - 关节4 --> -pi/2
arm.set_joint({JOINT1: 0, JOINT2: -pi/2, JOINT3:pi/2, JOINT4:-pi/2}, wait=True)
# 等待1s
time.sleep(1)
# 设置单个关节的弧度
# - 关节1 --> pi/3
arm.set_joint({JOINT1: math.pi/3})

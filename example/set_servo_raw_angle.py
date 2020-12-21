'''
设置机械臂关节舵机的原始角度
* 作者: 阿凯
* Email: xingshunkai@qq.com
* 更新时间: 2020-02-18
'''
# 添加路径
import sys
sys.path.append('../src/')
# 导入依赖
import time
from fs_arm_4dof import Arm4DoF
from config import *

# 机械臂初始化
arm = Arm4DoF()
# 设置机械臂舵机的原始角度
# * 关节1对应的#0舵机 --> -45°
# * 关节2对应的#1舵机 -->  20°
# * 关节3对应的#2舵机 --> -45°
# * 关节4对应的#3舵机 -->   0°
arm.set_servo_angle({JOINT1: -45, JOINT2: 20, JOINT3:-45, JOINT4:0}, wait=True)
# 等待1s
time.sleep(1)
# 单独设置关节1的舵机角度为0°
arm.set_servo_angle({JOINT1: 0}, wait=True)

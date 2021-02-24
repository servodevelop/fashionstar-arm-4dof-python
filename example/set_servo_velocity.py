'''
调整舵机的平均转速,延时不同转速下的效果
* 作者: 阿凯
* Email: kyle.xing@fashionstar.com.hk
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
# 设置#0舵机角度为0°
arm.set_servo_angle({JOINT1: 0}, wait=True)

# 设置舵机的平均转速为20, 单位是dps(度/秒)
arm.set_servo_velocity(20)
# 设置#0舵机角度为－45°
arm.set_servo_angle({JOINT1: -45}, wait=True)
# 等待1s
time.sleep(1)

# 设置舵机的平均转速为150, 单位是dps(度/秒)
arm.set_servo_velocity(150)
# 设置#0舵机角度为0°
arm.set_servo_angle({JOINT1: 0}, wait=True)


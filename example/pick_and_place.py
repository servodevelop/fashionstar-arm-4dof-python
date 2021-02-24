'''
使用气泵完成对木块的搬运
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
from micro_pump import MicroPump
from config import *

# 创建机械臂对象
arm = Arm4DoF()
# 创建气泵对象
pump = MicroPump()

# 吸头运动到木块的正上方
arm.move([12, 6, 4])
# 等待1s
time.sleep(1)
# 吸头落下
arm.move([12, 6, 0.5])
# 气泵开启
pump.on()
# 抬起物块
arm.move([12, 6, 4])
# 等待1s
time.sleep(1)
# 将木块抬至目的地的正上方
arm.move([12, -6, 4])
# 等待1s
time.sleep(1)
# 落下,让木块接近平面
arm.move([12, -6, 1])
# 释放气泵
pump.off()
# 吸头抬起
arm.move([12, -6, 4])
# 初始化机械臂的位姿
arm.init_pose()

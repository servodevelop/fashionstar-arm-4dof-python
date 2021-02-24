'''
测试移动机械臂末端执行器的位置
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
# 设置末端执行器(气泵吸头)的位置运动到 x=5cm y=0cm z=5cm
arm.move([5, 0, 5], wait=True)
# 等待1s
time.sleep(1)
# 设置末端执行器(气泵吸头)的位置运动到 x=5cm y=5cm z=10cm
arm.move([5, 5, 10], wait=True)

'''
机械臂几何标定-16点法(designed by 阿凯)

'''
import math # 数学计算
import yaml # YAML配置文件读取
import pandas # Excel数据读取
import numpy as np # 矩阵运算
from scipy import optimize as opt # 科学计算数值最优化
from scipy.optimize import minimize
from matplotlib import pyplot as plt # 可视化
from mpl_toolkits.mplot3d import Axes3D  # 空间三维画图

##########################################
## 1. 载入标定数据
##########################################
config_path = "../config/arm4dof.yaml" # 载入arm的配置文件
calib_data_path = '机械臂几何标定-十六点舵机原始角度数据.xlsx' # 标定数据的路径
# calib_data_path = '机械臂几何标定样本采集.xlsx' # 标定数据的路径
arm_cali_output_path = '../config/joint_calibration.yaml'

# 读入机械臂的配置文件
arm = None # 注意这里的arm是一个dict对象
with open(config_path, 'r', encoding="utf-8") as f:
    arm = yaml.load(f.read(),  Loader=yaml.SafeLoader)

# 载入连杆长度
link23 = arm['link']['link23']
link34 = arm['link']['link34']
tool_name = 'pen'
link45 = arm['tool'][tool_name]['link45']
link56 = arm['tool'][tool_name]['link56']


# 读入标定数据
raw_data_df = pandas.read_excel(calib_data_path)
# 提取DataFrame里面的数据，转换为ndarray
data_np = raw_data_df.iloc[:, 1:].to_numpy()

# 标准网格点位的坐标
grid_x_std = raw_data_df['x坐标'].to_numpy()
grid_y_std = raw_data_df['y坐标'].to_numpy()
grid_z_std = raw_data_df['z坐标'].to_numpy()
# 当theta1 = 0时，x,y -> x0
grid_x0_std = np.sqrt(np.power(grid_x_std,2) + np.power(grid_y_std, 2))

# 舵机原始角度
servo0_angle =  raw_data_df['舵机0'].to_numpy()
servo1_angle =  raw_data_df['舵机1'].to_numpy()
servo2_angle =  raw_data_df['舵机2'].to_numpy()
servo3_angle =  raw_data_df['舵机3'].to_numpy()

# 标定点的个数
n_points = len(grid_x_std)
##########################################
## 2.根据两点法，获取k_i, 跟b_i的参考值(初始值)
##########################################

# 计算关节1的标定系数
theta1_n90 = arm['joint']['joint2servo_data']['theta1_n90']
theta1_p90 = arm['joint']['joint2servo_data']['theta1_p90']
k1_ref = (theta1_p90 - theta1_n90) / (90.0 - (-90.0))
b1_ref = theta1_p90 - k1_ref*90.0
# 计算关节2的标定系数
theta2_n90 = arm['joint']['joint2servo_data']['theta2_n90']
theta2_p0 = arm['joint']['joint2servo_data']['theta2_p0']
k2_ref = (theta2_p0 - theta2_n90) / (0.0 - (-90.0))
b2_ref = theta2_p0 - k2_ref*0.0
# 计算关节3的标定系数
theta3_p0 = arm['joint']['joint2servo_data']['theta3_p0']
theta3_p90 = arm['joint']['joint2servo_data']['theta3_p90']
k3_ref = (theta3_p90 - theta3_p0) / (90.0 - (0.0))
b3_ref = theta3_p90 - k3_ref*90.0
# 计算关节4的标定系数
theta4_n90 = arm['joint']['joint2servo_data']['theta4_n90']
theta4_p90 = arm['joint']['joint2servo_data']['theta4_p90']
k4_ref = (theta4_p90 - theta4_n90) / (90.0 - (-90.0))
b4_ref = theta4_p90 - k4_ref*90.0

##########################################
## 3.计算关节1的标定数据
##########################################
# 计算关节1的标定值
joint1_angle = np.zeros_like(grid_x_std)
for i in range(n_points):
    joint1_angle[i] = np.degrees(math.atan2(grid_y_std[i], grid_x_std[i]))
# 线性拟合
k1, b1 = np.polyfit(joint1_angle, servo0_angle, 1)
# 计算关节1的弧度
theta1 = ((servo0_angle - b1) / k1)/180.0*np.pi

plt.plot([-50, 50], [k1*(-50)+b1,k1*(50)+b1], alpha=0.5)
plt.scatter(joint1_angle, servo0_angle, alpha=0.4)

plt.show()

##########################################
## 4.计算关节2,3,4的标定数据
##########################################
# 设定k_2-4, b_2-4的初始值
k2 = k2_ref
b2 = b2_ref
k3 = k3_ref
b3 = b3_ref
k4 = k4_ref
b4 = b4_ref

def servo2joint(k1, b1, k2, b2, k3, b3, k4, b4):
    '''舵机角度转换为关节弧度'''
    global servo0_angle
    global servo1_angle
    global servo2_angle
    global servo3_angle
    theta1 = ((servo0_angle - b1) / k1)/180.0*np.pi
    theta2 = ((servo1_angle - b2) / k2)/180.0*np.pi
    theta3 = ((servo2_angle - b3) / k3)/180.0* np.pi
    theta4 = ((servo3_angle - b4) / k4)/180* np.pi
    return theta1, theta2, theta3, theta4

def forward_kinematics(theta1, theta2, theta3, theta4):
    '''正向运动学'''
    global link23, link34, link45, link56
    theta23 = theta2 + theta3
    theta234 = theta23 + theta4
    fk_x0 = link23*np.cos(theta2) + link34*np.cos(theta23) + link45*np.cos(theta234) - link56*np.sin(theta234)
    fk_z = -link23*np.sin(theta2) - link34*np.sin(theta23) - link45*np.sin(theta234) -  link56*np.cos(theta234)
    fk_x = fk_x0 * np.cos(theta1)
    fk_y = fk_x0 * np.sin(theta1)
    return fk_x, fk_y, fk_z

# k4 = k4_ref
# b4 = b4_ref

def residual(params):
    '''目标函数(最小化)'''
    # global k1, b1
    # global k4, b4
    # 角度转弧度
    # k2, b2, k3, b3, k4, b4 = params
    k1, b1, k2, b2, k3, b3, k4, b4  = params
    theta1, theta2, theta3, theta4 = servo2joint(k1, b1, k2, b2, k3, b3, k4, b4)
    fk_x, fk_y, fk_z = forward_kinematics(theta1, theta2, theta3, theta4)
    return  np.sum(np.power(grid_x_std - fk_x, 2) + np.power(grid_y_std - fk_y, 2) + np.power(grid_z_std - fk_z, 2))

# 数值最优化
result = opt.minimize(residual, [k1, b1, k2_ref, b2_ref, k3_ref, b3_ref, k4_ref, b4_ref], method='BFGS')
# result = opt.minimize(residual, [k2_ref, b2_ref, k3_ref, b3_ref], method='BFGS')
k1, k2, k2, b2, k3, b3, k4, b4  = result.x


##########################################
## 5. 打印日志 保存文件
##########################################
# 打印日志
print("[标定参数参考值(优化前)]\n k1:{:.4f}\n b1:{:.4f}\n k2= {:.4f}\n b2= {:.4f}\n k3= {:.4f}\n b3= {:.4f}\n k4= {:.4f} \n b4= {:.4f}\n".format(\
    k1_ref, b1_ref, k2_ref, b2_ref, k3_ref, b3_ref, k4_ref, b4_ref))
print("[标定参数优化后]\n k1:{:.4f}\n b1:{:.4f}\n k2= {:.4f}\n b2= {:.4f}\n k3= {:.4f}\n b3= {:.4f}\n k4= {:.4f} \n b4= {:.4f}\n".format(k1, b1, k2, b2, k3, b3, k4, b4))

# 保存标定数据 Yaml
arm_cali_data = {
    'k1': float(k1),
    'b1': float(b1),
    'k2': float(k2),
    'b2': float(b2),
    'k3': float(k3),
    'b3': float(b3),
    'k4': float(k4),
    'b4': float(b4)
}
with open(arm_cali_output_path, 'w') as f:
    yaml.dump(arm_cali_data, f)

##########################################
## 6. 参数评估
##########################################
# 计算用参考标定系数(初始值)计算的xyz
fk_x_ref, fk_y_ref, fk_z_ref = forward_kinematics(*servo2joint(k1_ref, b1_ref, k2_ref, b2_ref, k3_ref, b3_ref, k4_ref, b4_ref))
# 使用标定后的系数计算的xyz
fk_x_opt, fk_y_opt, fk_z_opt = forward_kinematics(*servo2joint(k1, b1, k2, b2, k3, b3, k4, b4))
# 计算平均误差
err_mean_ref = np.mean(np.sqrt(np.power(grid_x_std - fk_x_ref, 2) +  np.power(grid_y_std - fk_y_ref, 2) + np.power(grid_z_std - fk_z_ref, 2)))
err_mean_opt = np.mean(np.sqrt(np.power(grid_x_std - fk_x_opt, 2) +  np.power(grid_y_std - fk_y_opt, 2) + np.power(grid_z_std - fk_z_opt, 2)))
print("标定前的平均误差 {:.4f} cm".format(err_mean_ref))
print("标定后的平均误差 {:.4f} cm".format(err_mean_opt))

##########################################
## 7. 标定结果可视化
##########################################

# 创建画布
fig = plt.figure()
ax = Axes3D(fig)
# 绘制网格
grid_x_unique = np.unique(grid_x_std)
grid_y_unique = np.unique(grid_y_std)
grid_z = grid_z_std[0]
for y in grid_y_unique:
    ax.plot3D(grid_x_unique, np.ones_like(grid_x_unique)*y, np.ones_like(grid_x_unique)*grid_z, c='gray')
for x in grid_x_unique:
    ax.plot3D(np.ones_like(grid_y_unique)*x, grid_y_unique, np.ones_like(grid_y_unique)*grid_z, c='gray')

# 绘制散点图
pts_std = ax.scatter(grid_x_std, grid_y_std, grid_z_std, c='orange') # 绘制标准网格
pts_before_cali = ax.scatter(fk_x_ref, fk_y_ref, fk_z_ref, c='red')
pts_after_cali = ax.scatter(fk_x_opt, fk_y_opt, fk_z_opt, c='green')

# 添加标签
plt.legend(handles=[pts_std, pts_before_cali, pts_after_cali],labels=['standard','before cali', 'after cali'],loc='best')

ax.set_zlim3d(-13, -4)
plt.show()

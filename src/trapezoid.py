import math
from math import sqrt,pow
import numpy as np
from matplotlib import pyplot as plt
import matplotlib as mpl

# 定义常量
REF_MAX_A = 2600.0 # 电机旋转时的最大角加速度 °/s^2
REF_MAX_W = 195.0 # 电机最大的转速 °/s
DELTA_T = 0.001 # 生成轨迹序列的时间间隔

def trapezoid_logging(theta_s, theta_e, w_s, w_e, w_max, a_max, t01, t12,t23):
    '''打印梯形加减速轨迹规划的结果'''
    print('初始位置: theta_s = {:.2f}°\n终止位置: theta_e = {:.2f}°'.format(theta_s, theta_e))
    print('初始速度: w_s = {:.2f} °/s \n终止速度: w_e = {:.2f}°/s'.format(w_s, w_e))
    print('最大速度: w_max = {:.2f} °/s'.format(w_max))
    print('最大加速度: a_max = {:.2f} °/s^2'.format(a_max))
    print('时间段: T1 = {:.2f} s  T2 = {:.2f}  T3 = {:.2f} \n总时间: T = {:.2f}'.format(t01, t12, t23, t01+t12+t23))

def trapezoid_sequence(theta_s:float, theta_e:float, t01:float, t12:float, t23:float,
                       w_s:float=0.0, w_e:float=0.0, w_max:float=REF_MAX_W, \
                       a_max:float=REF_MAX_A, t0:float=0.0):
    '''生成梯形轨迹规划的序列'''
    # 生成关键时刻的时间戳
    t1 = t0 + t01
    t2 = t1 + t12
    t3 = t2 + t23
    # 序列数据
    t_arr = [] # 时间
    a_arr = [] # 加速度 a
    w_arr = [] # 转速 w
    theta_arr = [] # 位置(角度) theta
    # PART1: 匀加速度段[t0, t1]
    for t in np.arange(t0, t1+DELTA_T, DELTA_T):
        t_arr.append(t)
        a_arr.append(a_max)
        w_arr.append(w_s + (t-t0)*a_max)
        theta_arr.append(theta_s + w_s*(t-t0) + 0.5*a_max*(t-t0)**2)
    # PART2: 匀速段[t1, t2]
    theta_t1 = theta_arr[-1]
    if t12 != 0:
        for t in np.arange(t1+DELTA_T, t2+DELTA_T, DELTA_T):
            t_arr.append(t)
            a_arr.append(0)
            w_arr.append(w_max)
            theta_arr.append(theta_t1 + w_max*(t-t1))
    # PART3: 匀减速段[t2, t3]
    theta_t2 = theta_arr[-1]
    for t in np.arange(t2+DELTA_T, t3+DELTA_T, DELTA_T):
        t_arr.append(t)
        a_arr.append(-a_max)
        w_arr.append(w_max - a_max*(t-t2))
        theta_arr.append(theta_t2 + w_max*(t-t2)-0.5*a_max*(t-t2)**2)
    
    t_arr = np.float32(t_arr)
    a_arr = np.float32(a_arr)
    w_arr = np.float32(w_arr)
    theta_arr = np.float32(theta_arr)
    
    return t_arr, a_arr, w_arr, theta_arr

def trapezoid_plan1(theta_s:float, theta_e:float, w_s:float=0.0, w_e:float=0.0,  w_max:float=REF_MAX_W, a_max:float=REF_MAX_A):
    '''计算梯形轨迹规划中能够达到的最大速度'''
    has_t12 = False # 是否存在匀速段T2
    dtheta = theta_e - theta_s # 角度位移
    direct =  1 if dtheta > 0 else -1 # 角度方向　是不是由小变大
    # 调整速度与加速度的正负
    a_max = direct*a_max if a_max*direct < 0 else a_max
    w_max = w_max*direct if w_max*direct < 0 else w_max
    # 假设达到最大速度之后的S1与S3之和
    # 注:这里其实也说明了, w_s与w_e的正负并不影响最终的角度位移
    s13 = (0.5/a_max)*(2*w_max**2 - w_s**2 - w_e**2)
    # 判断是否存在匀速段
    if direct*dtheta > direct*s13:
        has_t12 = True # 存在匀速段, 且可以达到参考最大速度
    else:
        # 不能达到最大的速度, 所以需要重新计算w_max
        w_max = direct*sqrt(dtheta*a_max + 0.5*(w_s**2 + w_e**2))
        # 重新计算s13
        s13 = 0.5*a_max*(2*w_max**2 - w_s**2 - w_e**2)
    
    # 计算梯形加减速的每个时间段
    t01 = (w_max-w_s)/a_max
    t12 =  (dtheta-s13)/w_max if has_t12 else 0
    t23 = (w_max-w_e)/a_max
    
    return w_max, a_max,t01,t12,t23

def trapezoid_plan2(theta_s:float, theta_e:float, t_target:float, \
                    w_s:float=0.0, w_e:float=0.0,  w_max:float=REF_MAX_W, a_max:float=REF_MAX_A):
    '''梯形轨迹规划算法2'''
    # theta的差值
    dtheta = theta_e-theta_s
    
    # 首先使用算法1判断是否会超出限制
    w_max, a_max,t01,t12,t23 = trapezoid_plan(theta_s, theta_e, w_s=w_s, w_e=w_e,\
                                              w_max=REF_MAX_W, a_max=REF_MAX_A)
    t_sum = t01 + t12 + t23 # 计算总时间
    if t_sum >= t_target:
        # 处理情况1跟情况2
        if t_sum > t_target:
            print('[WARN] 超出了加速度最大值a_max 或速度最大值w_max的限制')
            print('[WARN] T_sum={:.2f} > T_target={:.2f}'.format(t_sum, t_target))
        else:
            print('[INFO] 时间刚好与算法1规划的相同')
        return w_max, a_max,t01,t12,t23
    else:
        # 处理情况3,求解一元二次不等式
        a = 1
        b = -(w_s + w_e + t_target*a_max)
        c = 0.5*(w_s**2 + w_e**2) + a_max*dtheta
        # 判断等式是否有解
        if (b**2 - 4*a*c) < 0:
            print('[ERROR] 数值错误, 关于w_max的一元二次不等式没有解')
            return w_max, a_max,t01,t12,t23
        # 求解w_max的两根根
        tmp_v = math.sqrt(b**2-4*a*c)
        w_max_candi_root = [(-b+tmp_v)/(2*a),(-b-tmp_v)/(2*a)] # w_max候选的根
        print('w_max一元二次方程的两个根 {}'.format(w_max_candi_root))
        # 根据绝对值是否满足速度范围以及方向进行筛选
        w_max_candi_root = list(filter(lambda w: abs(w) < REF_MAX_W and w*dtheta >= 0, w_max_candi_root))
        if len(w_max_candi_root) == 0:
            print('[ERROR] w_max没有合法的根')
            return w_max, a_max,t01,t12,t23
        
        # 重新进行规划与计算
        w_max = w_max_candi_root[0] # 选取第一个
        t01 = (w_max - w_s) / a_max
        t23 = (w_max - w_e) / a_max
        t12 = t_target - (t01 + t23)
        # print('[INFO]成功求解新的w_max:{:.2f}'.format(w_max))
        return w_max, a_max, t01, t12, t23
    
def trapezoid_draw(t01, t12, t23, t_arr, a_arr, w_arr, theta_arr, t0:float=0):
    '''绘制梯形曲线'''
    # 生成关键时刻的时间戳
    t1 = t0 + t01
    t2 = t1 + t12
    t3 = t2 + t23
    
    plt.figure(figsize=(15, 3.5))
    plt.subplot(131)
    plt.title(r'$\theta$-$t$', size=20)
    plt.plot(t_arr, theta_arr, color='black')
    lower_b = min(theta_arr)-10
    upper_b = max(theta_arr)+10
    # 绘制分割线
    plt.plot([t0, t0], [lower_b, upper_b], linestyle='--', alpha=0.4, color='red')
    plt.plot([t1, t1], [lower_b, upper_b], linestyle='--', alpha=0.4, color='red')
    plt.plot([t2, t2], [lower_b, upper_b], linestyle='--', alpha=0.4, color='red')
    plt.plot([t3, t3], [lower_b, upper_b], linestyle='--', alpha=0.4, color='red')
    # 添加标注
    plt.xticks([t0, t1, t2, t3], [r'$t_0$', r'$t_1$', r'$t_2$', r'$t_3$'], size=15)
    plt.yticks([theta_arr[0], theta_arr[-1]], [r'$\theta_s$', r'$\theta_e$'], size=15)
    
    plt.subplot(132)
    plt.title(r'$w$-$t$', size=20)
    plt.plot(t_arr, w_arr, color='black')
    lower_b = min(w_arr)-20
    upper_b = max(w_arr)+20
    # 绘制分割线
    plt.plot([t0, t0], [lower_b, upper_b], linestyle='--', alpha=0.4, color='red')
    plt.plot([t1, t1], [lower_b, upper_b], linestyle='--', alpha=0.4, color='red')
    plt.plot([t2, t2], [lower_b, upper_b], linestyle='--', alpha=0.4, color='red')
    plt.plot([t3, t3], [lower_b, upper_b], linestyle='--', alpha=0.4, color='red')
    # 添加标注
    plt.xticks([t0, t1, t2, t3], [r'$t_0$', r'$t_1$', r'$t_2$', r'$t_3$'], size=15)
    w_tick_posi = [0, w_max]
    w_tick_name = [r'$0$', r'$w_{max}$']
    if abs(w_arr[0]) > 10:
        w_tick_name.append(r'$w_s$')
        w_tick_posi.append(w_arr[0])
        
    if abs(w_arr[-1]) > 10:
        w_tick_name.append(r'$w_e$')
        w_tick_posi.append(w_arr[-1])
        
    plt.yticks(w_tick_posi, w_tick_name, size=15)
    
    plt.subplot(133)
    plt.title(r'$a$-$t$', size=20)
    plt.plot(t_arr, a_arr, color='black')
    lower_b = min(a_arr)-200
    upper_b = max(a_arr)+200
    # 绘制分割线
    plt.plot([t0, t0], [lower_b, upper_b], linestyle='--', alpha=0.4, color='red')
    plt.plot([t1, t1], [lower_b, upper_b], linestyle='--', alpha=0.4, color='red')
    plt.plot([t2, t2], [lower_b, upper_b], linestyle='--', alpha=0.4, color='red')
    plt.plot([t3, t3], [lower_b, upper_b], linestyle='--', alpha=0.4, color='red')
    # 添加标注
    plt.xticks([t0, t1, t2, t3], [r'$t_0$', r'$t_1$', r'$t_2$', r'$t_3$'], size=15)
    plt.yticks([a_max, -a_max], [r'$A_{max}$', r'$-A_{max}$'], size=15)

def trapezoid_draw_velocity(axes, t01, t12, t23, t_arr, w_arr,t0:float=0,color:str='black', label:str='name'):
    '''绘制梯形曲线'''
    # 生成关键时刻的时间戳
    t1 = t0 + t01
    t2 = t1 + t12
    t3 = t2 + t23
    
    axes.set_title(r'$w$-$t$', size=20)
    axes.plot(t_arr, w_arr, color=color, label=label)
    lower_b = min(w_arr)-20
    upper_b = max(w_arr)+20
    # 绘制分割线
    axes.plot([t0, t0], [lower_b, upper_b], linestyle='--', alpha=0.2, color=color)
    axes.plot([t1, t1], [lower_b, upper_b], linestyle='--', alpha=0.2, color=color)
    axes.plot([t2, t2], [lower_b, upper_b], linestyle='--', alpha=0.2, color=color)
    axes.plot([t3, t3], [lower_b, upper_b], linestyle='--', alpha=0.2, color=color)
    axes.plot([t0, t3], [0, 0], alpha=1, color='black')
    # 添加标注
    t_list = [t0, t1, t2, t3]
    t_names = ['{:.3f}'.format(t) for t in t_list]
    plt.xticks(t_list, t_names, size=15,rotation=45)
    plt.xlabel(r'$t$', size=15)
    plt.ylabel(r'$w$', size=15)
    
    w_tick_posi = [0, w_max]
    w_tick_name = [r'$0$', r'$w_{max}$']
    if abs(w_arr[0]) > 10:
        w_tick_name.append(r'$w_s$')
        w_tick_posi.append(w_arr[0])
        
    if abs(w_arr[-1]) > 10:
        w_tick_name.append(r'$w_e$')
        w_tick_posi.append(w_arr[-1])
    plt.yticks(w_tick_posi, size=15)
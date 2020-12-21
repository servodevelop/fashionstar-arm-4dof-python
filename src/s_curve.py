from math import sqrt,pow
import numpy as np
from matplotlib import pyplot as plt

REF_T1 = 0.075 # T1 = t1- t0, 单位 s 
REF_T5 = 0.1 # T5 = t5 - t4, 单位 s
REF_MAX_W = 195 # 最大角速度 °/s
REF_MAX_A_ACC = REF_MAX_W/REF_T1 # 最大加速度 °/s^2
REF_MAX_A_DEC = REF_MAX_W/REF_T5 # 最大减速度 °/s^2
REF_MAX_J_ACC = REF_MAX_A_ACC / REF_T1 # 最大加加速度 °/s^3
REF_MAX_J_DEC = REF_MAX_A_DEC / REF_T5 # 最大加减速度 °/s^3

DELTA_T = 0.001 # 计算轨迹的时间间隔 单位s


def S(w_max, w_s, w_e, J_acc, J_dec):
    '''根据最大转速w_max, 计算S1+S3'''    
    s1 = (w_s + w_max) * sqrt(abs((w_max - w_s)/J_acc))
    s3 = (w_e + w_max) * sqrt(abs((w_max - w_e)/J_dec))
    return s1+s3

def scurve_find_w_max(s, w_s, w_e, J_acc, J_dec, iter_num=50):
    '''使用二分法,寻找合适的最大速度w_max
    @params s:
        总路程 s
    @params w_s:
        起始速度 如果w_s跟目标旋转方向同向则为正值,反之为负值
    @params w_e:
        末尾速度 如果w_e跟目标旋转方向同向则为正值,反之为负值
    @params J_acc：
        加加速
    @params J_dec:
        加减速
    @params iter_num:
        迭代求解的次数
    '''
    # 根据路程正负,修正Jerk
    J_acc = -1*J_acc if s < 0 and J_acc > 0 else J_acc
    J_dec = -1*J_dec if s < 0 and J_dec > 0 else J_dec
    
    # 能够达到最大速度, 可能存在匀速段
    if s > 0:
        if s >= S(REF_MAX_W, w_s, w_e, J_acc, J_dec):
            return REF_MAX_W
    else:
        if s <= S(-REF_MAX_W, w_s, w_e, J_acc, J_dec):
            return -REF_MAX_W
    
    # 没有匀速段, 通过二分法查找合适的最大速度
    left_w = 0
    right_w = REF_MAX_W if s > 0 else -1*REF_MAX_W
    
    ntime = 0 # 迭代次数
    angle_err = 0 # 角度误差/路程误差
    
    for i in range(iter_num):
        ntime = i+1
        # 计算中间节点
        mid_w = (left_w+right_w)/2
        angle_err = s - S(mid_w, w_s, w_e, J_acc, J_dec)
        # 舵机精度为0.1度, 所以这里设置的分辨率为0.01度
        if abs(angle_err) < 1e-2:
            break
        
        if s > 0:
            if angle_err < 0: 
                right_w = mid_w
            else:
                left_w = mid_w
        else:
            if angle_err > 0: 
                right_w = mid_w
            else:
                left_w = mid_w
        
    print('w_max: {:.2f} °/s, 迭代次数: {} mid_err: {:.4f}°'.format(mid_w, ntime, angle_err))
    return (left_w+right_w)/2

def scurve_time_plan(theta_s, theta_e, w_s, w_e, J_acc, J_dec):
    '''S曲线加减速, 时间规划'''
    s = theta_e - theta_s # 计算路程
    # 根据初末速度是否跟路程同向,判断正负
    # _w_s = w_s if s*w_s > 0 else -w_s
    # _w_e = w_e if s*w_e > 0 else -w_e
    #     if s < 0:
    #         J_acc *= -1
    #         J_dec *= -1
        
    # 注: 这里求得的最大速度都是大于0的
    w_max = scurve_find_w_max(s, w_s, w_e, J_acc, J_dec)
    
    T1 = sqrt(abs((w_max - w_s)/J_acc))
    T5 = sqrt(abs((w_max - w_e)/J_dec))
    T4 = (s-S(w_max, w_s, w_e, J_acc, J_dec))/w_max
    
    return T1, T4, T5

def scurve_key_status(theta_s, theta_e, w_s, w_e, J_acc, J_dec, T1, T4, T5):
    '''五段式非对称S曲线,关键时间段的状态+S曲线合法判断'''
    ret = True # S曲线是否合法
    
    # 根据T1,T5以及J_acc,J_dec推导最大加速度
    A_acc = J_acc * T1 # 计算最大加速度
    A_dec = J_dec * T5 # 计算最大减速度
    
    # 如果theta_s > theta_e A,J的数值都需要反转
    if theta_s > theta_e:
        J_acc = -1*J_acc if J_acc > 0 else J_acc
        J_dec = -1*J_dec if J_dec > 0 else J_dec
        A_acc = -1*A_acc if A_dec > 0 else A_acc
        A_dec = -1*A_dec if A_dec > 0 else A_dec
    
    # 计算关键时刻的速度(单位 °/s)
    w_t1 = w_s + 0.5*J_acc*T1**2 # 计算t1时刻的角速度,
    w_t3 = w_s + A_acc * T1 # 计算t3时刻的角速度
    w_t4 = w_t3  # 计算t4时刻的角速度
    w_t5 = w_t4 - 0.5*J_dec*T5**2 # 计算t5时刻的角速度
    # w_t7 = w_s + A_acc*T1 - A_dec*T5 # 计算t5时刻的角速度,一般默认为0
    w_t7 = w_t5 - A_dec*T5 + 0.5*J_dec*T5**2
    w_max = w_t3 # 最大转速
    
    # 速度约束检查
    # w_s + A_acc*T1 = w_e + A_dec*T5 = w_max
    if abs(w_t7-w_e) > 1e-1:
        # w_s + A_acc*T1 = w_e + A_dec*T5 = w_max
        print('[ERROR] 速度约束不匹配 w(t_7) = {:.3f} °/s , w_e = {:.3f}°/s'.format(w_t7, w_e))
        ret = False
    
    # 检查最大速度约束
    if (abs(w_max)-REF_MAX_W > 1e-1):
        print('[ERROR] 最大速度超过最大速度约束 w_max:{:.3f} °/s, REF_MAX_W = {:.3f}°/s'.format(w_max,REF_MAX_W))
        ret = False
        
    # 计算关键时刻的位置(单位　°)
    theta_t1 = theta_s + w_s*T1 + 1.0/6.0*J_acc*T1**3
    theta_t3 = theta_t1 + w_t1*T1 + 0.5*A_acc*T1**2  - 1.0/6.0*J_acc*T1**3
    theta_t4 = theta_t3 + w_max*T4
    theta_t5 = theta_t4 + w_max*T5 - 1.0/6.0*J_dec*T5**3
    theta_t7 = theta_t5 + w_t5*T5 - 0.5*A_dec*T5**2 + 1.0/6.0*J_dec*T5**3
    
    # 位置约束
    # theta_e - theta_s = w_s*T1 +　w_max*(T1+T4+T5) + w_e*T_5
    if abs(theta_t7 - theta_e) > 1e-1:
        print('[ERROR] 位置约束不匹配 theta(t_7)={:.3f}  theta_e={:.3f}'.format(theta_t7, theta_e))
        ret = False
        
    return ret, J_acc, J_dec, A_acc, A_dec, w_t1, w_t3, w_t4, w_t5, w_t7, theta_t1, theta_t3, theta_t4, theta_t5, theta_t7

def scurve_trajectory_seq(theta_s, theta_e, w_s, w_e, J_acc, J_dec, T1, T4, T5):
    '''S曲线轨迹规划 生成序列'''    
    # 获取S曲线关键时刻的状态
    results = scurve_key_status(theta_s, theta_e, w_s, w_e, J_acc, J_dec, T1, T4, T5)
    # 将数据解包
    ret, J_acc, J_dec, A_acc, A_dec, w_t1, w_t3, w_t4, w_t5, w_t7, theta_t1, theta_t3, theta_t4, theta_t5, theta_t7 = results
    if not ret:
        print('[ERROR]S曲线非法')
        # return ret, [],[],[],[],[]
        
    # 计算关键时间节点
    t0 = 0
    t1 = t0 + T1
    t3 = t1 + T1
    t4 = t3 + T4
    t5 = t4 + T5
    t7 = t5 + T5
    
    t_arr = [] # 时间
    j_arr = [] # 加加速度 Jerk
    a_arr = [] # 加速度 a
    w_arr = [] # 转速 w
    theta_arr = [] # 位置(角度) theta
    
    # 加加速段 [0,t1]
    for t in np.arange(0, t1+DELTA_T, DELTA_T):
        t_arr.append(t)
        j_arr.append(J_acc)
        a_arr.append(J_acc*(t-t0))
        w_arr.append(w_s+0.5*J_acc*(t-t0)**2)
        theta_arr.append(theta_s + w_s*(t-t0) + 1.0/6.0*J_acc*(t-t0)**3)
    # 减加速度段 (t1,t3]
    for t in np.arange(t1+DELTA_T, t3+DELTA_T, DELTA_T):
        t_arr.append(t)
        j_arr.append(-J_acc)
        a_arr.append(A_acc-J_acc*(t-t1))
        w_arr.append(w_t1 + A_acc*(t-t1)-0.5*J_acc*(t-t1)**2)
        theta_arr.append(theta_t1 + w_t1*(t-t1) + 0.5*A_acc*(t-t1)**2 - 1.0/6.0*J_acc*(t-t1)**3)
    # 匀速段 (t3, t4]
    for t in np.arange(t3+DELTA_T, t4+DELTA_T, DELTA_T):
        t_arr.append(t)
        j_arr.append(0)
        a_arr.append(0)
        w_arr.append(w_t3)
        theta_arr.append(theta_t3 + w_t3*(t-t3))   
    # 加减速段 (t4, t5]
    for t in np.arange(t4+DELTA_T, t5+DELTA_T, DELTA_T):
        t_arr.append(t)
        j_arr.append(-J_dec)
        a_arr.append(-J_dec*(t-t4))
        w_arr.append(w_t4-0.5*J_dec*(t-t4)**2)
        theta_arr.append(theta_t4 + w_t4*(t-t4) - 1.0/6.0 * J_dec*(t-t4)**3)
    # 减减速段
    for t in np.arange(t5+DELTA_T, t7+DELTA_T, DELTA_T):
        t_arr.append(t)
        j_arr.append(J_dec)
        a_arr.append(-A_dec + J_dec*(t-t5))
        w_arr.append(w_t5-A_dec*(t-t5)+0.5*J_dec*(t-t5)**2)
        theta_arr.append(theta_t5 + w_t5*(t-t5) - 0.5*A_dec*(t-t5)**2 + 1.0/6.0*J_dec*(t-t5)**3)
    return ret, t_arr, j_arr, a_arr, w_arr, theta_arr

def draw_scurve(t_arr, j_arr, a_arr, w_arr, theta_arr):
    '''绘制S曲线'''
    plt.figure(figsize=(8, 20))
    plt.subplot(411)
    plt.title('theta-t')
    plt.plot(t_arr, theta_arr)

    plt.subplot(412)
    plt.title('w-t')
    plt.plot(t_arr, w_arr)

    plt.subplot(413)
    plt.title('a-t')
    plt.plot(t_arr, a_arr)

    plt.subplot(414)
    plt.title('jerk-t')
    plt.plot(t_arr, j_arr)

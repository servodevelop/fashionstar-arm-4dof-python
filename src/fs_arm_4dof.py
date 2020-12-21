'''
Fashion Star 4自由度机械臂Python SDK
--------------------------------------------------
- 作者: 阿凯
- Email: xingshunkai@qq.com
- 更新时间: 2020-03-11
--------------------------------------------------
'''
import time # 时间
import logging # 日志管理
import yaml
import os.path
import math
from math import sin,cos,atan2,acos,degrees,radians,sqrt
import serial # 串口通信
from fs_uservo import UartServoManager # 导入串口舵机管理器
from minimum_jerk import minimum_jerk_plan, minimum_jerk_seq # 轨迹规划算法
import numpy as np

# 设置日志等级
# logging.basicConfig(level=logging.INFO)
class Arm4DoF:
    def __init__(self, device:str=None, is_init_pose:bool=True):
        '''机械臂初始化'''
        # 创建串口对象
        try:
            # 载入配置文件
            self.load_config()
            if device is None:
                device = self.config['uart']['port']
            # 创建UART对象
            self.uart = serial.Serial(
                port=device, \
                baudrate=self.config['uart']['baudrate'],\
                parity=self.config['uart']['parity'], \
                stopbits=self.config['uart']['stopbits'],\
                bytesize=self.config['uart']['bytesize'],\
                timeout=self.config['uart']['timeout'])
            
            # 创建串口舵机管理器
            self.uservo = UartServoManager(self.uart, srv_num=self.joint_num, mean_dps=self.config['joint']['speed']['default'])
            # 初始化位姿
            if is_init_pose:
                self.init_pose()
            # 切换当前的工具
            self.switch_tool(self.config['tool']['default'])

        except serial.SerialException as serial_err:
            logging.error('该设备不存在,请重新填写UART舵机转接板的端口号')
        except Exception as e:
            logging.error("Error: {} --".format(str(e)))

    def load_config(self, config_path:str="./config/arm4dof.yaml"):
        '''载入配置文件'''
        with open(config_path, 'r', encoding="utf-8") as f:
            self.config = yaml.load(f.read(),  Loader=yaml.SafeLoader)
        # 获取关节个数
        self.joint_num = self.config['joint']['joint_num']
        # 关节ID号 (同时也是数据的序号)
        self.joint1 = self.config['joint']['joint1']['servo_id']
        self.joint2 = self.config['joint']['joint2']['servo_id']
        self.joint3 = self.config['joint']['joint3']['servo_id']
        self.joint4 = self.config['joint']['joint4']['servo_id']
        # 读取角度约束
        # 关节角度下限(单位 弧度)
        self.theta_lowerb = [
            radians(self.config['joint']['joint1']['angle_min']),
            radians(self.config['joint']['joint2']['angle_min']),
            radians(self.config['joint']['joint3']['angle_min']),
            radians(self.config['joint']['joint4']['angle_min'])]
        # 关节角度上限(单位 弧度)
        self.theta_upperb =  [
            radians(self.config['joint']['joint1']['angle_max']),
            radians(self.config['joint']['joint2']['angle_max']),
            radians(self.config['joint']['joint3']['angle_max']),
            radians(self.config['joint']['joint4']['angle_max'])]
        # 载入连杆尺寸
        self.link23 = self.config['link']['link23']
        self.link34 = self.config['link']['link34']
        
        
        # 运动控制
        self.delta_t = self.config['motion_control']['delta_t']
        self.rad_per_step = self.config['motion_control']['rad_per_step']
        # 载入关节标定系数
        self.calc_joint2servo()

    def switch_tool(self, tool_name):
        '''切换工具'''
        if tool_name not in self.config['tool']:
            logging.warn("failed to change tool, unkown tool {}".format(tool_name))
            return False
        # 设置工具
        self.tool = tool_name
        # 更新连杆长度
        self.link45 = self.config['tool'][self.tool]['link45']
        self.link56 = self.config['tool'][self.tool]['link56']
        # 接下来的参数是为了加速逆向运动学做的预计算
        self.link23_pow2 = self.link23*self.link23
        self.link34_pow2 = self.link34*self.link34

        # 根据工具的类型, 定义不同的初始化
        if tool_name == 'pump':
            self.pump_init() # 气泵初始化
        # TODO 添加爪子的
        return True

    def calc_joint2servo(self, joint_cali_path='./config/joint_calibration.yaml'):
        '''计算关节角度到舵机原始角度的线性变换的系数'''
        # 判断配置文件里是否存在joint_calibration.yaml这个文件
        # 如果有的话，直接从这里面读取k b
        
        if os.path.exists(joint_cali_path):
            with open(joint_cali_path, 'r', encoding="utf-8") as f:
                joint_cali_data = yaml.load(f.read(),  Loader=yaml.SafeLoader)
                self.joint2servo_k = [joint_cali_data['k1'],joint_cali_data['k2'], joint_cali_data['k3'], joint_cali_data['k4']]
                self.joint2servo_b = [joint_cali_data['b1'],joint_cali_data['b2'], joint_cali_data['b3'], joint_cali_data['b4']]
            return

        # 计算关节1的标定系数
        theta1_n90 = self.config['joint']['joint2servo_data']['theta1_n90']
        theta1_p90 = self.config['joint']['joint2servo_data']['theta1_p90']
        k1 = (theta1_p90 - theta1_n90) / (90.0 - (-90.0))
        b1 = theta1_p90 - k1*90.0
        # 计算关节2的标定系数
        theta2_n90 = self.config['joint']['joint2servo_data']['theta2_n90']
        theta2_p0 = self.config['joint']['joint2servo_data']['theta2_p0']
        k2 = (theta2_p0 - theta2_n90) / (0.0 - (-90.0))
        b2 = theta2_p0 - k2*0.0
        # 计算关节3的标定系数
        theta3_p0 = self.config['joint']['joint2servo_data']['theta3_p0']
        theta3_p90 = self.config['joint']['joint2servo_data']['theta3_p90']
        k3 = (theta3_p90 - theta3_p0) / (90.0 - (0.0))
        b3 = theta3_p90 - k3*90.0
        # 计算关节4的标定系数
        theta4_n90 = self.config['joint']['joint2servo_data']['theta4_n90']
        theta4_p90 = self.config['joint']['joint2servo_data']['theta4_p90']
        k4 = (theta4_p90 - theta4_n90) / (90.0 - (-90.0))
        b4 = theta4_p90 - k4*90.0
        # 将标定数据填入到列表里
        self.joint2servo_k = [k1, k2, k3, k4]
        self.joint2servo_b = [b1, b2, b3, b4]

    def init_pose(self):
        '''机械臂位姿初始化'''
        # 从配置文件里面读取初始角度
        theta1 = radians(self.config['joint']['joint1']['angle_default'])
        theta2 = radians(self.config['joint']['joint2']['angle_default'])
        theta3 = radians(self.config['joint']['joint3']['angle_default'])
        theta4 = radians(self.config['joint']['joint4']['angle_default'])
        self.set_joint([theta1, theta2, theta3, theta4], wait=False, interval=2000)
        time.sleep(2)
        # 读取当前的角度
        self.uservo.query_all_srv_angle()
        
    def set_servo_velocity(self, speed:float):
        '''设置舵机的转速 单位°/s'''
        lowerb = self.config['joint']['speed']['min']
        upperb = self.config['joint']['speed']['max']
        self.uservo.mean_dps = max(min(abs(speed), upperb),lowerb)

    def set_servo_angle(self, angles, wait:bool=False):
        '''设置舵机的原始角度'''
        if type(angles) == list:
            for srv_idx, angle in enumerate(angles):
                logging.info('设置舵机角度, 舵机#{} 目标角度 {:.1f}'.format(srv_idx, angle))
                self.uservo.set_srv_angle(int(srv_idx), float(angle))
        elif type(angles) == dict:
            for srv_idx, angle in angles.items():
                logging.info('设置舵机角度, 舵机#{} 目标角度 {:.1f}'.format(srv_idx, angle))
                self.uservo.set_srv_angle(int(srv_idx), float(angle))
        if wait:
            self.wait() # 等待舵机角度到达目标角度

    def set_joint(self, thetas, wait:bool=False, interval=None):
        '''设置关节的弧度'''
        if type(thetas) == list:
            for srv_idx, theta in enumerate(thetas):
                # 检查弧度的范围约束
                theta = min(max(self.theta_lowerb[srv_idx], theta), self.theta_upperb[srv_idx])
                logging.info('设置关节弧度, 关节#{}  弧度 {:.4f} 角度 {:.1f}'.format(srv_idx+1, theta, math.degrees(theta)))
                # 弧度转角度 
                theta = degrees(theta)
                # 根据关节的弧度计算出舵机的原始角度
                angle = self.joint2servo_k[srv_idx]*theta + self.joint2servo_b[srv_idx] 
                self.uservo.set_srv_angle(int(srv_idx), float(angle), interval=interval)
        elif type(thetas) == dict:
            for srv_idx, theta in thetas.items():
                 # 检查弧度的范围约束
                theta = min(max(self.theta_lowerb[srv_idx], theta), self.theta_upperb[srv_idx])
                logging.info('设置关节弧度, 关节#{} 弧度 {:.4f} 角度 {:.1f}'.format(srv_idx+1, theta, math.degrees(theta)))
                # 弧度转角度 
                theta = degrees(theta)
                # 根据关节的弧度计算出舵机的原始角度
                angle = self.joint2servo_k[srv_idx]*theta + self.joint2servo_b[srv_idx] 
                self.uservo.set_srv_angle(int(srv_idx), float(angle), interval=interval)
        if wait:
            self.wait() # 等待舵机角度到达目标角度

    def trajectory_plan(self, joint_id:int, theta_e:float, T:float, w_s:float=0.0, w_e:float=0.0, a_s:float=0, a_e:float=0):
        '''Minimum Jerk轨迹规划'''
        # 获取当前关节的
        theta_s = self.get_thetas()[joint_id]
        c = minimum_jerk_plan(theta_s, theta_e, w_s, w_e, a_s, a_e, T)
        t_arr, theta_arr = minimum_jerk_seq(T, c, delta_t=self.delta_t)
        return t_arr, theta_arr
    
    def set_joint_minjerk(self, thetas, T:float):
        '''设置关节弧度2-带MinimumJerk 轨迹规划版本,需要阻塞等待.
        关节空间下的
        '''
        # 将thetas转换为dict类型
        if type(thetas) == list:
            thetas_arr = thetas
            thetas = {}
            for joint_id, theta in enumerate(thetas_arr):
                thetas[joint_id] = theta
        # theta序列
        theta_seq_dict = {}
        
        t_arr = None # 时间序列
        # 生成轨迹序列
        for joint_id, theta_e in thetas.items():
            t_arr, theta_arr = self.trajectory_plan(joint_id, theta_e, T)
            # print("joint{} theta_arr: {}".format(joint_id, theta_arr))
            # print(theta_arr)
            theta_seq_dict[joint_id] = theta_arr # np.copy(theta_arr)
        
        # 按照轨迹去执行
        i = 0
        tn = len(t_arr)
        while True:
            if i >= tn:
                break
            t_start = time.time()
            next_thetas = {}
            for joint_id in theta_seq_dict.keys():
                next_thetas[joint_id] = theta_seq_dict[joint_id][i]
            # 设置关节弧度
            self.set_joint(next_thetas, interval=0)
            # print('t_i={} next_thetas: {}'.format(i, next_thetas))
            t_end = time.time()
            # print('t_end - t_start = {:.4f}'.format(t_end-t_start))
            # time.sleep(self.delta_t-(t_end - t_start)) # 延迟
            m = int(math.floor((t_end - t_start) / self.delta_t))
            i += (1 + m)
            # 补齐所需延迟等待的时间
            time.sleep(self.delta_t - ((t_end - t_start) - m*self.delta_t))
        return theta_seq_dict
    
    def set_damping(self, power:int=0):
        '''设置阻尼模式'''
        for jnt_idx in range(self.joint_num):
            self.uservo.set_damping(jnt_idx, power=power)

    def forward_kinematics(self, thetas:list):
        '''机械臂正向运动学'''
        theta1, theta2, theta3, theta4 = thetas
        # 为了减少计算量，提前计算角度之和
        theta23 = theta2 + theta3
        theta234 = theta23 + theta4
        pitch = theta234

        xe0 = self.link23*cos(theta2) + self.link34*cos(theta23) + self.link45*cos(theta234)
        xe0 -= self.link56*sin(pitch)
        xe = xe0*cos(theta1) 
        ye = xe0*sin(theta1)

        ze = -self.link23*sin(theta2) - self.link34*sin(theta23) - self.link45*sin(theta234)
        ze -= self.link56 *cos(pitch)
        
        return [xe, ye, ze], pitch 

    def inverse_kinematics(self, p_tool:list, pitch:float=0.0):
        '''逆向运动学'''
        # 提取末端的位置
        xe, ye, ze = p_tool
        # 计算theta1
        theta1 = atan2(ye, xe) # 获得关节1的弧度
        if theta1 < self.theta_lowerb[self.joint1] or theta1 > self.theta_upperb[self.joint1]:
            logging.warning('theta1={}, 超出了关节1角度范围'.format(degrees(theta1)))
            return False,None
        # theta1 = 0时的末端坐标
        xe0 = sqrt(xe*xe + ye*ye) 
        # 计算Joint5的位置
        x5 = xe0 + self.link56*sin(pitch)
        z5 = ze + self.link56*cos(pitch)
        # 计算Joint4的位置
        x4 = x5 - self.link45*cos(pitch)
        z4 = z5 + self.link45*sin(pitch)
        # 计算theta2
        beta = atan2(z4, x4)
        d_pow2 = z4*z4 + x4*x4
        d = sqrt(d_pow2)
        cos_phi = (self.link23_pow2 + d_pow2 - self.link34_pow2)/(2*self.link23*d)
        if abs(cos_phi) > 1:
            logging.warning('cos_theta2={}, Value Error'.format(cos_phi))
            return False,None
        phi = acos(cos_phi)

        theta2 = - (beta + phi)
        # 检验theta2是否合法
        if theta2 < self.theta_lowerb[self.joint2] or theta2 > self.theta_upperb[self.joint2]:
            logging.warning('theta2={}, 超出了关节2角度范围'.format(degrees(theta2)))
            return False,None
        # 计算Joint3
        x3 = self.link23*cos(theta2)
        z3 = - self.link23*sin(theta2)
        theta3 = -theta2 - atan2(z4-z3, x4-x3)
        # 检验theta3是否合法
        if theta3 < self.theta_lowerb[self.joint3] or theta3 > self.theta_upperb[self.joint3]:
            logging.warning('theta3={}, 超出了关节3角度范围'.format(degrees(theta3)))
            return False,None
        theta4 = pitch - (theta2+theta3)
        # 检验theta4是否合法
        if theta4 < self.theta_lowerb[self.joint4] or theta4 > self.theta_upperb[self.joint4]:
            logging.warning('theta4={}, 超出了关节3角度范围'.format(degrees(theta4)))
            return False,None
        return True, [theta1, theta2, theta3, theta4]

    def get_servo_angles(self, theta_type:type = list):
        '''获取当前关节的舵机原始角度'''
        self.uservo.query_all_srv_angle() # 查询舵机角度
        srv_angles = []
        for srv_idx in range(self.joint_num):
            srv_angles.append(self.uservo.servos[srv_idx].angle)
        if theta_type == dict:
            srv_angle_dict = {}
            for srv_id, srv_angle in enumerate(srv_angles):
                srv_angle_dict[srv_id] = srv_angle
                return srv_angle_dict
        return srv_angles
        
    def get_thetas(self, theta_type:type = list):
        '''获取当前关节的弧度'''
        self.uservo.query_all_srv_angle() # 查询舵机角度
        thetas = []
        for srv_idx in range(self.joint_num):
            # 线性变换
            theta = (self.uservo.servos[srv_idx].angle - self.joint2servo_b[srv_idx]) / self.joint2servo_k[srv_idx]
            # 角度转换为弧度
            theta = radians(theta)
            thetas.append(theta)
        if theta_type == dict:
            # 转换为list格式
            thetas_dict = {}
            for joint_id, theta in enumerate(thetas):
                thetas_dict[joint_id] = theta
            return thetas_dict
        else:
            return thetas

    def get_tool_pose(self):
        '''获取机械臂在笛卡尔坐标系下的位置'''
        # 查询关节的弧度
        thetas = self.get_thetas()
        # 正向运动学
        return self.forward_kinematics(thetas)

    def move_p2p_simple(self, p_tool:list, pitch:float=0.0, wait:bool=False, is_linear:bool=True):
        '''控制机械臂的末端旋转到特定的位置(利用舵机内置的梯形轨迹规划)'''
        ret, thetas = self.inverse_kinematics(p_tool, pitch=pitch)
        if ret:
            logging.info('移动机械臂末端到: {}'.format(p_tool))
            self.set_joint(thetas, wait=wait)
        else:
            logging.warn('机械臂末端不能到达: {}'.format(p_tool))

    def move_p2p_minjerk(self,p_tool:list, pitch:float, T:float=1.0, wait:bool=False):
        '''使用minimum jerk测试机械臂的运动'''
        # 更新关节角度
        self.get_thetas()
        ret, thetas = self.inverse_kinematics(p_tool, pitch=pitch)
        if not ret:
            logging.error('超出工作空间, 机械臂末端到达不了{}'.format(p_tool))
            return False
        # 关节空间下的MinmumJerk
        self.set_joint_minjerk(thetas, T)
    
    
    def move_p2p_minjerk_in_tasksapce(self, p_tool:list, pitch:float, T:float):
        '''在任务空间下完成(适合快速搬运，对轨迹质量要求不高)'''
        # 先判断目标点是否可以达到
        ret, thetas = self.inverse_kinematics(p_tool, pitch=pitch)
        if not ret:
            logging.error('超出工作空间, 机械臂末端到达不了{}'.format(p_tool))
            return False 
        # 获取机械臂当前的位姿
        [x_start, y_start, z_start], pitch_start = self.get_tool_pose()
        # 设置目标位姿
        x_end, y_end, z_end = p_tool
        pitch_end = pitch
        # Minimum Jerk 在Task Space下进行规划
        # 临时先这么写 初末速度与加速度都为0
        c_x = minimum_jerk_plan(x_start, x_end, 0, 0, 0, 0, T)
        t_arr_x, x_arr = minimum_jerk_seq(T, c_x, delta_t=self.delta_t)
        c_y = minimum_jerk_plan(y_start, y_end, 0, 0, 0, 0, T)
        _, y_arr = minimum_jerk_seq(T, c_y, delta_t=self.delta_t)
        c_z = minimum_jerk_plan(z_start, z_end, 0, 0, 0, 0, T)
        _, z_arr = minimum_jerk_seq(T, c_z, delta_t=self.delta_t)
        c_pitch = minimum_jerk_plan(pitch_start, pitch_end, 0, 0, 0, 0, T)
        t_arr, pitch_arr = minimum_jerk_seq(T, c_pitch, delta_t=self.delta_t)
        # 映射到JointSpace下
        theta_seq_dict = {}
        for joint_id in range(self.joint_num):
            theta_seq_dict[joint_id] = []
        t_arr = []
        for i in range(len(t_arr_x)):
            tool_posi = [x_arr[i], y_arr[i], z_arr[i]]
            pitch = pitch_arr[i]
            ret, thetas = self.inverse_kinematics(tool_posi, pitch)
            if ret:
                t_arr.append(t_arr_x[i])
                for joint_id in range(self.joint_num):
                    theta_seq_dict[joint_id].append(thetas[joint_id])
        i = 0
        tn = len(t_arr)
        while True:
            if i >= tn:
                break
            t_start = time.time()
            next_thetas = {}
            for joint_id in theta_seq_dict.keys():
                next_thetas[joint_id] = theta_seq_dict[joint_id][i]
            # 设置关节弧度
            self.set_joint(next_thetas, interval=0)
            t_end = time.time()
            cur_block_interval = self.delta_t
            if i > 0:
                cur_block_interval = t_arr[i] - t_arr[i-1]
            time_delay = cur_block_interval - (t_end - t_start)
            while time_delay < 0:
                i += 1
                time_delay += (t_arr[i] - t_arr[i-1])
                
            time.sleep(time_delay)
            i += 1
                
    def excute_path(self, t_arr, theta_seq_dict):
        '''执行轨迹'''
        i = 0
        tn = len(t_arr)
        while True:
            if i >= tn:
                break
            t_start = time.time()
            next_thetas = {}
            for joint_id in theta_seq_dict.keys():
                next_thetas[joint_id] = theta_seq_dict[joint_id][i]
            # 设置关节弧度
            self.set_joint(next_thetas, interval=0)
            t_end = time.time()
            cur_block_interval = self.delta_t
            if i > 0:
                cur_block_interval = t_arr[i] - t_arr[i-1]
            time_delay = cur_block_interval - (t_end - t_start)
            while time_delay < 0:
                i += 1
                time_delay += (t_arr[i] - t_arr[i-1])
                
            time.sleep(time_delay)
            i += 1

    def move_line(self, p_tool:list, pitch:float):
        '''末端走直线(梯形加减速算法)'''
        # 先判断目标点是否可以达到
        ret, thetas = self.inverse_kinematics(p_tool, pitch=pitch)
        if not ret:
            logging.error('超出工作空间, 机械臂末端到达不了{}'.format(p_tool))
            return False 
        # 获取机械臂当前的位姿
        [x_start, y_start, z_start], pitch_start = self.get_tool_pose()
        # 设置目标位姿
        x_end, y_end, z_end = p_tool
        pitch_end = pitch

        # 计算差值
        dx = x_end - x_start
        dy = y_end - y_start
        dz = z_end - z_start
        dpitch = pitch_end - pitch_start
        distance = dx*dx  + dy*dy + dz*dz
        # print("distance^2: {}".format(distance))
        distance = math.sqrt(distance)
        step_size = 0.10 # step的尺寸
        n_step= math.floor(distance / step_size)

        t_arr = []
        theta_seq_dict = {}
        for joint_id in range(self.joint_num):
            theta_seq_dict[joint_id] = []

        for i in range(1, n_step + 1):
            ratio = i/n_step
            cur_x = x_start + ratio * dx
            cur_y = y_start + ratio * dy
            cur_z = z_start + ratio * dz
            cur_pitch = pitch_start + ratio*dpitch
            # self.move_p2p_simple([cur_x, cur_y, cur_z], cur_pitch)
            # 逆向运动学查看是否有解
            ret, thetas = self.inverse_kinematics([cur_x, cur_y, cur_z], cur_pitch)
            if ret:
                t_arr.append(0.005 * i)
                for joint_id in range(self.joint_num):
                    theta_seq_dict[joint_id].append(thetas[joint_id])
        self.excute_path(t_arr, theta_seq_dict)
        
    # 工具控制相关的逻辑
    def pump_init(self):
        '''气泵初始化'''
        self.pump_off()

    def pump_on(self):
        '''气泵开启'''
        if self.tool != 'pump':
            # 切换工具为气泵
            self.switch_tool('pump')
        servo_id = self.config['tool']['pump']['servo_id']
        self.uservo.set_srv_angle(servo_id, -90, interval=0, power=0)
            
    def pump_off(self):
        '''气泵关闭'''
        if self.tool != 'pump':
            # 切换工具为气泵
            self.switch_tool('pump')
        servo_id = self.config['tool']['pump']['servo_id']
        # 气泵关闭
        self.uservo.set_srv_angle(servo_id, 0, interval=0, power=0)
        time.sleep(0.2)
        # 电磁阀打开
        self.uservo.set_srv_angle(servo_id, 90, interval=0, power=0)
        time.sleep(0.2)
        # 电磁阀关闭
        self.uservo.set_srv_angle(servo_id, 0, interval=0, power=0)
    
    def wait(self):
        '''机械臂等待'''
        self.uservo.wait()

if __name__ == "__main__":
    arm = Arm4DoF()

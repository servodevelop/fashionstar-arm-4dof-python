'''
Fashion Star 串口舵机Python SDK
--------------------------------------------------
- 作者: 阿凯
- Email: xingshunkai@qq.com
- 更新时间: 2020-03-11
--------------------------------------------------
'''
import time
import logging
import serial
import struct

# 设置日志等级
# logging.basicConfig(level=logging.INFO)

class Packet:
    '''数据包'''
    # 使用pkt_type来区分请求数据还是响应数据
    PKT_TYPE_REQUEST = 0 # 请求包
    PKT_TYPE_RESPONSE = 1 # 响应包
    HEADER_LEN = 2 # 帧头校验数据的字节长度
    HEADERS = [b'\x12\x4c', b'\x05\x1c']
    CODE_LEN = 1 # 功能编号长度
    SIZE_LEN = 1 # 字节长度
    CHECKSUM_LEN = 1 # 校验和长度

    @classmethod
    def calc_checksum(cls, code, param_bytes=b'', pkt_type=1):
        '''计算校验和'''
        header = cls.HEADERS[pkt_type]
        return sum(header + struct.pack('<BB', code, len(param_bytes)) + param_bytes) %256

    @classmethod
    def verify(cls, packet_bytes, pkt_type=1):
        '''检验数据是否合法'''
        # 获取帧头
        header = cls.HEADERS[pkt_type]
      
        # 帧头检验
        if packet_bytes[:cls.HEADER_LEN] != cls.HEADERS[pkt_type]:
            return False
        code, size = struct.unpack('<BB', packet_bytes[cls.HEADER_LEN : cls.HEADER_LEN + cls.CODE_LEN + cls.SIZE_LEN])
        
        # 长度校验
        param_bytes = packet_bytes[cls.HEADER_LEN + cls.CODE_LEN + cls.SIZE_LEN : -cls.CHECKSUM_LEN]
        if len(param_bytes) != size:
            return False

        # 校验和检验
        checksum = packet_bytes[-cls.CHECKSUM_LEN]
        # logging.info('实际的Checksum : {} 计算得到的Checksum: {}'.format(checksum, cls.calc_checksum(code , param_bytes, pkt_type=pkt_type)))
        
        # 校验和检查
        if checksum != cls.calc_checksum(code , param_bytes, pkt_type=pkt_type):
            return False
        
        # 数据检验合格
        return True

    @classmethod
    def pack(cls, code, param_bytes=b''):
        '''数据打包为二进制数据'''
        size = len(param_bytes)
        checksum = cls.calc_checksum(code, param_bytes, pkt_type=cls.PKT_TYPE_REQUEST)
        frame_bytes = cls.HEADERS[cls.PKT_TYPE_REQUEST] + struct.pack('<BB', code, size) + param_bytes + struct.pack('<B', checksum)
        return frame_bytes
    
    @classmethod
    def unpack(cls, packet_bytes):
        '''二进制数据解包为所需参数'''
        if not cls.verify(packet_bytes, pkt_type=cls.PKT_TYPE_RESPONSE):
            # 数据非法
            return None
        code = struct.unpack('<B', packet_bytes[cls.HEADER_LEN:cls.HEADER_LEN+cls.CODE_LEN])[0]
        param_bytes = packet_bytes[cls.HEADER_LEN + cls.CODE_LEN + cls.SIZE_LEN : -cls.CHECKSUM_LEN]
        return code, param_bytes

class PacketBuffer:
    '''Packet中转站'''
    def __init__(self, is_debug=True):
        self.is_debug = is_debug
        self.packet_bytes_list = []
        # 清空缓存区域
        self.empty_buffer()
    
    def update(self, next_byte):
        '''将新的字节添加到Packet中转站'''
        # logging.info('[INFO]: next byte: 0x%02x'%next_byte[0])
        if not self.header_flag:
            # 填充头部字节
            if len(self.header) < Packet.HEADER_LEN:
                # 向Header追加字节
                self.header += next_byte
                if len(self.header) == Packet.HEADER_LEN and self.header == Packet.HEADERS[Packet.PKT_TYPE_RESPONSE]:
                    self.header_flag = True
            elif len(self.header) == Packet.HEADER_LEN:
                # 首字节出队列
                self.header = self.header[1:] + next_byte
                # 查看Header是否匹配
                if self.header == Packet.HEADERS[Packet.PKT_TYPE_RESPONSE]:
                    # print('header: {}'.format(self.header))
                    self.header_flag = True
        elif not self.code_flag:
            # 填充Code
            if len(self.code) < Packet.CODE_LEN:
                self.code += next_byte
                if len(self.code) == Packet.CODE_LEN:
                    # print('code: {}'.format(self.code))
                    self.code_flag = True
        elif not self.size_flag:
            # 填充参数尺寸
            if len(self.size) < Packet.SIZE_LEN:
                self.size += next_byte
                if len(self.size) == Packet.SIZE_LEN:
                    self.size_flag = True
                    # 更新参数个数
                    self.param_len = struct.unpack('<B', self.size)[0]
        elif not self.param_bytes_flag:
            # 填充参数
            if len(self.param_bytes) < self.param_len:
                self.param_bytes += next_byte
                if len(self.param_bytes) == self.param_len:
                    self.param_bytes_flag = True
        else:
            # 计算校验和
            # 构建一个完整的Packet
            tmp_packet_bytes = self.header + self.code + self.size + self.param_bytes + next_byte
            
            ret = Packet.verify(tmp_packet_bytes, pkt_type=Packet.PKT_TYPE_RESPONSE)
            
            if ret:
                self.checksum_flag = True
                # 将新的Packet数据添加到中转列表里
                self.packet_bytes_list.append(tmp_packet_bytes)
            
            # 重新清空缓冲区
            self.empty_buffer()
        
    def empty_buffer(self):
        # 数据帧是否准备好
        self.param_len = None
        self.header = b''
        self.header_flag = False
        self.code = b''
        self.code_flag = False
        self.size = b''
        self.size_flag = False
        self.param_bytes = b''
        self.param_bytes_flag = False
    
    def has_valid_packet(self):
        '''是否有有效的包'''
        return len(self.packet_bytes_list) > 0
    
    def get_packet(self):
        '''获取队首的Bytes'''
        return self.packet_bytes_list.pop(0)


class UartServoInfo:
    '''串口舵机的信息'''
    SERVO_DEADBLOCK = 0.2 # 舵机死区(实际只有0.1°)
    SERVO_ANGLE_LOWERB = -135 # 舵机角度下限
    SERVO_ANGLE_UPPERB = 135 # 舵机角度上限 
    
    def __init__(self, id, lowerb=None, upperb=None):
        self.id = id # 舵机的ID
        self.cur_angle = None # 当前的角度
        self.target_angle = None # 目标角度
        self.is_online = False # 舵机是否在线 
        # 舵机角度上下限
        self.lowerb = lowerb if lowerb is not None else self.SERVO_ANGLE_LOWERB
        self.upperb = upperb if upperb is not None else self.SERVO_ANGLE_UPPERB

    def is_stop(self):
        '''判断舵机是否已经停止'''
        # 角度误差在死区范围以内则判断为已经到达目标点
        return abs(self.cur_angle - self.target_angle) <= self.SERVO_DEADBLOCK
    
    @property
    def angle(self):
        '''获取当前舵机的角度'''
        return self.cur_angle
    
    def move(self, angle):
        '''设置舵机的目标角度'''
        # 角度边界约束
        angle = self.lowerb if angle < self.lowerb else angle
        angle = self.upperb if angle > self.upperb else angle

        self.target_angle = angle 

    def update(self, angle):
        '''更新当前舵机的角度'''
        self.cur_angle = angle
    
    def __str__(self):
        return "目标角度:{:.1f} 实际角度:{:.1f} 角度误差:{:.2f}".format(self.target_angle, self.angle, self.target_angle-self.angle)

class UartServoManager:
    '''串口舵机管理器'''
    UPDATE_INTERVAL_MS = 10 # ms
    CODE_PING = 1 # 舵机检测
    CODE_QUERY_SERVO_ANGLE = 10 # 查询舵机的角度
    CODE_QUERY_SERVO_INFO = 5 # 查询舵机所有的信息 (未使用)
    CODE_SET_SERVO_ANGLE = 8 # 设置舵机角度
    CODE_SET_SPIN = 7 # 设置轮式模式
    CODE_SET_DAMPING = 9 # 设置阻尼模式
    CODE_SET_SERVO_ANGLE_BY_VELOCITY = 12 # 角度设置(带加减速算法)
    RESPONSE_CODE_NEGLECT = []
    # 定义轮式控制的四种控制方法
    WHEEL_MODE_STOP = 0x00 # 停止
    WHEEL_MODE_NORMAL = 0x01 # 常规模式
    WHEEL_MODE_ROUND = 0x02 # 定圈
    WHEEL_MODE_TIME = 0x03 # 定时

    def __init__(self, uart, srv_num=254, is_scan_servo=True, mean_dps=100):
        self.uart = uart
        self.pkt_buffer = PacketBuffer()
        self.mean_dps = mean_dps # 默认的舵机旋转角速度
        # 存放舵机信息
        self.servos = {}
        # 云台一共是三个舵机 编号从0-2
        # for servo_idx in range(srv_num):
        #    self.servos[servo_idx] = UartServoInfo(servo_idx)
        
        # 返回的CODE与函数的映射
        self.response_handle_funcs = {
            self.CODE_QUERY_SERVO_ANGLE: self.response_query_servo_angle,
            self.CODE_PING: self.response_ping,
        }
        
        # self.cur_ping_servo_id = 0 # 当前Ping的舵机ID号
        if is_scan_servo:
            self.scan_servo(srv_num=srv_num)

    def send_request(self, code, param_bytes):
        '''发送请数据'''
        packet_bytes = Packet.pack(code, param_bytes)
        try:
            self.uart.write(packet_bytes)
            # logging.info('串口发送请求数据 code:{}'.format(code))
            # logging.info('数据帧内容:')
            # logging.info(''.join(['0x%02x ' % b for b in packet_bytes]))
        except serial.SerialException as e:
            logging.error('串口数据发送异常, 请检查是否是USB口松动或设备号变更, 需重新初始化机械臂')
            # TODO 添加掉线检查
    def ping(self, servo_id:int):
        '''发送Ping请求'''
        # self.cur_ping_servo_id = servo_id #　为了可视化显示
        self.send_request(self.CODE_PING, struct.pack('<B', servo_id))
        
        logging.info('PING 舵机 id={}'.format(servo_id))

    def scan_servo(self, srv_num=254):
        '''扫描所有的舵机'''
        # ping所有的舵机
        for servo_idx in range(srv_num):
            # 发送ping请求
            self.ping(servo_idx)
            self.update(wait_response=True)
            if servo_idx in self.servos:
                logging.info('[fs_uservo]串口舵机ID={} 响应ping'.format(servo_idx))
        logging.info("有效的舵机ID列表: {}".format(list(self.servos.keys())))
    def response_ping(self, param_bytes):
        '''响应PING请求'''
        servo_id, = struct.unpack('<B', param_bytes)
        if servo_id not in self.servos:
            self.servos[servo_id] = UartServoInfo(servo_id)
            self.servos[servo_id].is_online = True # 设置舵机在线的标志位
            logging.info('[fs_uservo]ECHO 添加一个新的舵机 id={}'.format(servo_id))
        else:
            self.servos[servo_id].is_online = True # 设置舵机在线的标志位
            logging.info('[fs_uservo]ECHO 已知舵机 id={}'.format(servo_id))
        

    def query_servo_angle(self, servo_id):
        '''更新单个舵机的角度'''
        logging.info('查询单个舵机的角度 id={}'.format(servo_id))
        self.send_request(self.CODE_QUERY_SERVO_ANGLE, struct.pack('<B', servo_id))
        self.update(wait_response=True) # 等待数据回传
        return self.servos[servo_id].angle

    def query_all_srv_angle(self):
        '''更新所有的舵机角度'''
        for servo_id in self.servos:
            self.query_servo_angle(servo_id)
        
    def response_query_servo_angle(self, param_bytes):
        '''相应查询单个舵机角度'''
        # 数据解包
        servo_id, angle = struct.unpack('<Bh', param_bytes)
        # 舵机的分辨率是0.1度
        angle /= 10
        
        if servo_id not in self.servos:
            # 没有在已知的舵机列表里面
            # 添加一个新的舵机对象
            self.servos[servo_id] = UartServoInfo(servo_id, angle)
            self.servos[servo_id].is_online = True
            # self.servos[servo_id].cur_angle = angle
            self.servos[servo_id].update(angle)
            logging.info('[fs_uservo]添加一个新的舵机 id={}  角度:{:.2f} deg'.format(servo_id, angle))
        else:
            # 更新当前的角度
            # self.servos[servo_id].cur_angle = angle
            self.servos[servo_id].update(angle)
            logging.info('[INFO] 更新舵机角度 id={}  角度: {:.2f} deg'.format(servo_id, angle))

    def refresh_srv_list(self, max_servo_id=254):
        '''刷新当前的舵机列表'''
        # 清空已有的字典
        self.servos = {}
        for servo_idx in range(max_servo_id):
            self.ping(servo_idx)
            for ti in range(20):
                # 查询一个舵机最多等待1000ms
                self.update()
                if servo_idx in self.servos:
                    break
                # 每隔100ms查询一次
                utime.sleep_ms(50)
            
    def query_srv_info(self, servo_id):
        '''查询单个舵机的所有配置'''
        self.send_request(self.CODE_QUERY_SERVO_INFO, struct.pack('<B', servo_id))
        # logging.info('查询单个舵机的所有配置 id={}'.format(servo_id))
        self.update(wait_response=True)

    def set_srv_angle(self, servo_id, angle, interval=None, mean_dps=None, power=0):
        '''发送舵机角度控制请求
        @param servo_id 
            舵机的ID号
        @param angle 
            舵机的目标角度
        @param interval 
            中间间隔 单位ms
        @param mean_dps 
            平均转速 degree per second
        '''
        if servo_id not in self.servos:
            # logging.warn('未知舵机序号: {}'.format(servo_id))
            # 如果不在servos列表里面, 只发送控制指令
            angle = int(angle * 10)
            param_bytes = struct.pack('<BhHH', servo_id, angle, interval, power)
            self.send_request(self.CODE_SET_SERVO_ANGLE, param_bytes)
            return True
        
        # 获取舵机信息
        srv_info = self.servos[servo_id]

        # 周期如果是0的话就意味着需要马上旋转到目标的角度
        # 如果不是0则需要计算所需的周期interval
        if interval != 0:
            if srv_info.cur_angle is None:
                # 初始状态还没有角度
                interval = 800
            elif interval is None and mean_dps is None:
                # 延时时间差不多是15ms旋转一度,可以比较平滑
                interval = int((abs(angle - srv_info.angle) / self.mean_dps) *1000)
            elif mean_dps is not None:
                # 根据mean_dps计算时间间隔 （转换为ms）
                interval = int((abs(angle - srv_info.angle) / mean_dps) *1000)
        
        # 同步修改srv_info
        self.servos[servo_id].move(angle)
        # 发送控制指令
        # 单位转换为0.1度
        angle = int(angle * 10)
        param_bytes = struct.pack('<BhHH', servo_id, angle, interval, power)
        self.send_request(self.CODE_SET_SERVO_ANGLE, param_bytes)
        return True

    def set_srv_angle_v(self, servo_id:int, angle:float, velocity:float, t_acc:float, t_dec:float, power:float=0):
        '''设置舵机角度(带加减速控制算法, MoveOnAngleModeExByVelocity)'''
        # 设置舵机的目标角度
        self.servos[servo_id].move(angle)
        # 数据加工
        angle = int(angle * 10)# 角度转换为0.1度
        velocity = int(velocity*10)# 转速单位变换为0.1°/s
        # 数据帧填充
        param_bytes = struct.pack('<BhHHHH', servo_id, angle, velocity, t_acc, t_dec, power)
        self.send_request(self.CODE_SET_SERVO_ANGLE_BY_VELOCITY, param_bytes)
        
    def set_spin(self, servo_id, mode, value=0, is_cw=True, speed=None):
        '''设置舵机轮式模式控制
        @param servo_id
            舵机的ID号
        @param mode
            舵机的模式 取值范围[0,3]
        @param value 
            定时模式下代表时间(单位ms)
            定圈模式下代表圈数(单位圈)
        ＠param is_cw
            轮子的旋转方向, is_cw代表是否是顺指针旋转
        @param speed
            轮子旋转的角速度, 单位 度/s
        '''
        # 轮式模式的控制方法
        method = mode | 0x80 if is_cw else mode
        # 设置轮子旋转的角速度
        speed = self.mean_dps if speed is None else speed
        
        self.send_request(self.CODE_SET_SPIN, struct.pack('<BBHH', servo_id, method,speed, value))

    def set_damping(self, servo_id, power=0):
        '''设置阻尼模式
        @param servo_id
            舵机ID
        @param power
            舵机保持功率
        '''
        self.send_request(self.CODE_SET_DAMPING, struct.pack('<BH', servo_id, power))
    
    def update(self, is_empty_buffer=False, wait_response=False, timeout=0.02):
        '''舵机管理器的定时器回调函数'''
        t_start = time.time() # 获取开始时间
        while True:
            # 读入所有缓冲区的Bytes
            buffer_bytes = self.uart.readall()
            
            if len(buffer_bytes) != 0:
                logging.info('Recv Bytes: ')
                logging.info(' '.join(['0x%02x'%b for b in buffer_bytes]))

            # 将读入的Bytes打包成数据帧
            if buffer_bytes is not None:
                for b in buffer_bytes:
                    self.pkt_buffer.update( struct.pack('<B', b))
            
            t_cur = time.time() # 获取当前的时间戳
            is_timeout = (t_cur - t_start) > timeout # 是否超时
            
            if not wait_response:
                break
            elif self.pkt_buffer.has_valid_packet() or is_timeout:
                if is_timeout:
                    logging.info('等待超时')
                break
        
        # 相应回调数据
        while self.pkt_buffer.has_valid_packet():
            # 处理现有的返回数据帧
            response_bytes = self.pkt_buffer.get_packet()
            # 解包
            code, param_bytes = Packet.unpack(response_bytes)
            # 根据code找到相应处理函数
            if code in self.response_handle_funcs:
                self.response_handle_funcs[code](param_bytes)
            else:
                logging.warn('未知功能码 : {}'.format(code))
        
        # 清空原来的缓冲区
        if is_empty_buffer:
            self.pkt_buffer.empty_buffer()
        
    def is_stop(self):
        '''判断所有的舵机是否均停止旋转'''
        for servo_id, srv_info in self.servos.items():
            if not srv_info.is_stop():
                return False
        return True
    
    def wait(self, timeout=None):
        '''等待舵机旋转到特定的角度'''
        t_start = time.time()
        while True:
            # 更新舵机角度
            self.query_all_srv_angle()
            if self.is_stop():
                break
            
            # 超时检测
            if timeout is not None:
                t_current = time.time()
                if t_current - t_start > timeout:
                    break
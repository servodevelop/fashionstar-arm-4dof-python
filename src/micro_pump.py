'''
气泵模块
'''
import time
from config import GPIO_MAGNETIC_SWITCH,GPIO_PUMP_MOTOR
from gpiozero import LED as PumpMotor
from gpiozero import LED as MagneticSwitch

class MicroPump:
    def __init__(self):
        self.motor = PumpMotor(GPIO_PUMP_MOTOR)
        self.switch = PumpMotor(GPIO_MAGNETIC_SWITCH)
        self.motor.off()
        self.switch.off()

    def on(self):
        '''气泵吸气'''
        # 气泵开
        self.motor.on()
        # 等待一段时间
        time.sleep(1.0)

    def off(self):
        '''气泵放气'''
        # 气泵关
        self.motor.off()
        time.sleep(0.3)
        # 放气过程 : 放气->延时->关闭->延时 (循环三次)
        for rpt_i in range(1):
            self.switch.on()
            time.sleep(0.3)
            self.switch.off()
            time.sleep(0.3)
        time.sleep(0.5)

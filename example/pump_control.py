'''
控制气泵吸气与放气

注: 气泵相关的逻辑取决于所运行的平台,若有变更需修改micro_bump.py
* 作者: 阿凯
* Email: xingshunkai@qq.com
* 更新时间: 2020-02-18
'''
# 添加路径
import sys
sys.path.append('../src/')
# 导入依赖
import time
from micro_pump import MicroPump

# 创建气泵对象
pump = MicroPump()
# 气泵开启,吸气
pump.on()
# 等待2s
time.sleep(2)
# 气泵关闭,放气
pump.off()

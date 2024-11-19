import time
from .communication import ProximitySwitchCommunication

class ProximitySwitchController:
    def __init__(self, communication):
        self.comm = communication
        
    def get_switch_status(self):
        """获取开关量输入状态"""
        status = self.comm.read_switch_status()
        # if status is not None:
        #     print(f"开关量输入状态: {status}")
        # else:
        #     print("无法获取开关状态。")
        return status

    def monitor_switch(self, interval=0.1):
        """持续监控开关状态"""
        try:
            while True:
                self.get_switch_status()
                time.sleep(interval)
        except KeyboardInterrupt:
            print("监控已停止。") 
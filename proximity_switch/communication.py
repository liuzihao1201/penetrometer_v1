import minimalmodbus
import serial

class ProximitySwitchCommunication:
    def __init__(self, port, slave_address=1):
        self.instrument = minimalmodbus.Instrument(port, slave_address)
        self.setup_communication()
        
    def setup_communication(self):
        """设置通信参数"""
        self.instrument.serial.baudrate = 9600
        self.instrument.serial.bytesize = 8
        self.instrument.serial.parity = serial.PARITY_NONE
        self.instrument.serial.stopbits = 1
        self.instrument.serial.timeout = 1
        self.instrument.mode = minimalmodbus.MODE_RTU
        self.instrument.clear_buffers_before_each_transaction = True
        
    def read_switch_status(self, address=630):
        """读取开关量输入状态"""
        try:
            register_value = self.instrument.read_register(address, 0)
            status = [(register_value >> i) & 1 for i in range(1)]  # 假设有1个输入点
            return status
        except IOError:
            print("读取失败，请检查连接和设置。")
            return None 
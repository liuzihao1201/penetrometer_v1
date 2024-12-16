import minimalmodbus
import serial

class SensorCommunication:
    def __init__(self, port, slave_address=1):
        self.instrument = minimalmodbus.Instrument(port, slave_address)
        self.setup_communication()
        
    def setup_communication(self):
        """设置通信参数"""
        self.instrument.serial.baudrate = 19200
        self.instrument.serial.bytesize = 8
        self.instrument.serial.parity = serial.PARITY_NONE
        self.instrument.serial.stopbits = 1
        self.instrument.serial.timeout = 0.2
        self.instrument.mode = minimalmodbus.MODE_RTU
        self.instrument.clear_buffers_before_each_transaction = True
        
    def read_registers(self, address, count, functioncode=3):
        """读取多个寄存器"""
        return self.instrument.read_registers(address, count, functioncode=functioncode)

    def write_registers(self, address, values):
        """写入多个寄存器"""
        self.instrument.write_registers(address, values)  # 写入多个寄存器

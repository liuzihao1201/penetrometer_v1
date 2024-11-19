import minimalmodbus
import serial
import logging
import time

class CutMotorCommunication:
    def __init__(self, port, slave_address=2):
        self.instrument = minimalmodbus.Instrument(port, slave_address)
        self.setup_communication()
        
    def setup_communication(self):
        """设置通信参数"""
        self.instrument.serial.baudrate = 9600
        self.instrument.serial.bytesize = 8
        self.instrument.serial.parity = serial.PARITY_EVEN
        self.instrument.serial.stopbits = 1
        self.instrument.serial.timeout = 0.2
        self.instrument.mode = minimalmodbus.MODE_RTU
        self.instrument.clear_buffers_before_each_transaction = True
        
    def test_connection(self):
        """测试通信连接"""
        return self.instrument.read_register(0x0028, functioncode=3)
        
    def write_register(self, address, value, functioncode=6):
        """写入单个寄存器"""
        self.instrument.write_register(address, value, functioncode=functioncode)
        
    def read_register(self, address, functioncode=3):
        """读取单个寄存器"""
        return self.instrument.read_register(address, functioncode=functioncode)
        
    def write_registers(self, address, values):
        """写入多个寄存器"""
        self.instrument.write_registers(address, values)

    def read_registers(self, address, count, functioncode=3):
        """读取多个寄存器"""
        return self.instrument.read_registers(address, count, functioncode=functioncode)

class MotorSerialManager:
    def __init__(self):
        self.connection = None

    def open_motor_connection(self, port):
        """打开电机串口连接"""
        try:
            self.connection = serial.Serial(port, baudrate=9600, timeout=1)
            return True, f"成功连接到 {port}"
        except Exception as e:
            return False, f"无法连接到 {port}: {str(e)}"

    def close_motor_connection(self):
        """关闭电机串口连接"""
        if self.connection and self.connection.is_open:
            try:
                self.connection.close()
                return True, "串口连接已断开"
            except Exception as e:
                return False, f"无法断开连接: {str(e)}"
        return False, "连接未打开" 
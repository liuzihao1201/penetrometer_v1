import struct
from .communication import SensorCommunication

class SensorController:
    def __init__(self, communication):
        self.comm = communication

    def monitor_2_sensor(self):
        """监控传感器数据"""
        try:
            # 读取寄存器数据
            data = self.comm.read_registers(0x0100, 4, functioncode=3)
            # 解析前两个寄存器为float型拉压力
            pulling_pressure = struct.unpack('>f', struct.pack('>HH', data[0], data[1]))[0]
            # 解析后两个寄存器为float型扭矩
            torque = struct.unpack('>f', struct.pack('>HH', data[2], data[3]))[0]
            return round(pulling_pressure, 2), round(torque, 2)
        except Exception as e:
            print(f"无法监控传感器数据: {str(e)}")
            return None, None

    def reset_pulling_pressure(self):
        """重置拉压力"""
        try:
            self.comm.write_registers(0x0320, [0x0000, 0x0001])
            print("拉压力已重置")
        except Exception as e:
            print(f"无法重置拉压力: {str(e)}")

    def reset_torque(self):
        """重置扭矩"""
        try:
            self.comm.write_registers(0x0320, [0x0000, 0x0002])
            print("扭矩已重置")
        except Exception as e:
            print(f"无法重置扭矩: {str(e)}")

    def reset_all(self):
        """重置所有数据"""
        try:
            self.comm.write_registers(0x0320, [0x0000, 0x000A])
            print("所有数据已重置")
        except Exception as e:
            print(f"无法重置所有数据: {str(e)}") 
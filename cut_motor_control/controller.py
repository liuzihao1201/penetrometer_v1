import time
import logging

class CutMotorController:
    # 寄存器地址定义
    REGISTERS = {
        'speed': 0x0040,  # 速度寄存器
        'mode': 0x0080,  # 模式寄存器
        'position_speed': 0x0046,  # 位置控制速度寄存器
        'position_type': 0x0047,  # 位置控制类型寄存器
        'position_target': 0x0048,  # 目标位置寄存器
        'direction': 0x0045,  # 方向寄存器
        'servo_mode': 0x00C1,  # 伺服模式寄存器
        'encoder_high': 0x002C,  # 编码器输入脉冲数量高半字
        'encoder_low': 0x002D,  # 编码器输入脉冲数量低半字
        'reset_data': 0x700A,  # 重置数据寄存器
    }
    
    def __init__(self, communication):
        self.comm = communication
        
    def set_speed(self, speed_rpm):
        """设置电机速度"""
        if not -10000 <= speed_rpm <= 10000:
            raise ValueError(f"速度值 {speed_rpm} rpm 超出范围(-10000 to 10000)")
        
        # 使用补码表示速度
        if speed_rpm < 0:
            speed_rpm = (1 << 16) + speed_rpm  # 将负数转换为补码形式
        
        self.comm.write_register(self.REGISTERS['speed'], speed_rpm)
        
        time.sleep(0.2)  # 等待0.2秒以确保命令生效
        return self.comm.read_register(self.REGISTERS['speed'])
        
    def stop_motor(self):
        """停止电机"""
        current_speed = self.comm.read_register(self.REGISTERS['speed'])
        if current_speed > 1000:
            self.comm.write_register(self.REGISTERS['speed'], 500)
            time.sleep(1)  # 等待1秒以逐步减速
            
        self.comm.write_register(self.REGISTERS['speed'], 0)
        time.sleep(0.5)  # 等待0.5秒以确保电机完全停止
        return self.comm.read_register(self.REGISTERS['speed'])
        
    def set_servo_mode(self, mode):
        """设置伺服控制方式"""
        if mode not in [0, 1, 2]:
            raise ValueError("无效的控制方式")
            
        self.comm.write_register(self.REGISTERS['servo_mode'], mode)
        time.sleep(0.1)  # 等待0.1秒以确保命令生效
        return self.comm.read_register(self.REGISTERS['servo_mode'])
        
    def set_position_control(self, speed_rpm, position_type, target_position):
        """设置位置控制"""
        if not 0 <= abs(speed_rpm) <= 10000:
            raise ValueError(f"速度值超出范围")
        if position_type not in [0, 1]:
            raise ValueError("无效的位置控制类型")
            
        # 设置速度和方向
        self.comm.write_register(self.REGISTERS['position_speed'], abs(speed_rpm))
        direction_value = 1 if speed_rpm < 0 else 0
        self.comm.write_register(self.REGISTERS['direction'], direction_value)
        
        # 设置位置控制类型
        self.comm.write_register(self.REGISTERS['position_type'], position_type)
        
        # 设置目标位置
        high_word = (target_position >> 16) & 0xFFFF
        low_word = target_position & 0xFFFF
        self.comm.write_registers(self.REGISTERS['position_target'], [high_word, low_word])
        
        return {
            'speed': self.comm.read_register(self.REGISTERS['position_speed']),
            'direction': self.comm.read_register(self.REGISTERS['direction']),
            'type': self.comm.read_register(self.REGISTERS['position_type']),
            'position': (self.comm.read_register(self.REGISTERS['position_target']) << 16) | 
                        self.comm.read_register(self.REGISTERS['position_target'] + 1)
        } 

    def get_encoder_pulses(self):
        """获取编码器输入脉冲数量，支持负数"""
        # 使用read_registers一次性读取高低半字
        high_low_words = self.comm.read_registers(self.REGISTERS['encoder_high'], 2, functioncode=3)
        high_word, low_word = high_low_words
        
        # 将高低字合并为一个32位整数
        pulses = (high_word << 16) | low_word
        
        # 检查是否为负数（如果高字的最高位为1，则表示负数）
        if high_word & 0x8000:  # 0x8000 是 16 位数的最高位
            pulses -= (1 << 32)  # 将其转换为负数（补码形式）
        
        return pulses

    def reset_data(self):
        """重置切割电机数据"""
        try:
            self.comm.write_register(self.REGISTERS['reset_data'], 1, functioncode=6)  # 写入寄存器以清零数据
            print("切割电机数据已重置")
        except Exception as e:
            print(f"无法重置切割电机数据: {str(e)}")
import minimalmodbus
from sensor_2.communication import SensorCommunication
from sensor_2.controller import SensorController
import time

# 创建与设备通信的函数
def communicate_with_device(device_address):
    # 初始化仪器
    instrument = minimalmodbus.Instrument('COM3', device_address)  # 端口名称, 从站地址
    instrument.serial.baudrate = 9600  # 波特率
    instrument.serial.timeout = 1  # 读取超时时间

    # 读取寄存器的示例（假设寄存器地址为0）
    try:
        response = instrument.read_register(0, 0)  # 寄存器地址, 小数位数
        print(f"Response from device {device_address}: {response}")
    except IOError:
        print(f"Failed to communicate with device {device_address}")

# 与设备1通信
communicate_with_device(1)

# 与设备2通信
communicate_with_device(2)

def test_monitor_2_sensor():
    # 创建通信对象
    sensor_comm = SensorCommunication(port='COM3', slave_address=1)  # 确保端口和地址正确
    # 创建控制器对象
    sensor_controller = SensorController(sensor_comm)
    
    try:
        while True:
            # 调用监控函数
            pulling_pressure, torque = sensor_controller.monitor_2_sensor()
            if pulling_pressure is not None and torque is not None:
                print(f"拉压力: {pulling_pressure} N, 扭矩: {torque} Nm")
            else:
                print("无法获取传感器数据")
            
            time.sleep(0.1)  # 每秒钟测量一次
    except KeyboardInterrupt:
        print("测量已停止")

# 测试监控功能
test_monitor_2_sensor()

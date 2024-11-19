import minimalmodbus

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

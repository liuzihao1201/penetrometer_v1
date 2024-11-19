from PySide6.QtWidgets import QApplication, QMainWindow, QGraphicsScene
from gui_v1 import Ui_Form
from cut_motor_control.communication import CutMotorCommunication
from cut_motor_control.controller import CutMotorController
from proximity_switch.communication import ProximitySwitchCommunication
from proximity_switch.control import ProximitySwitchController
from penetration_motor_control.communication import PenetrationMotorCommunication
from penetration_motor_control.controller import PenetrationMotorController
import serial.tools.list_ports
import threading
import time
from PySide6.QtCore import QTimer, Qt, QMargins
from PySide6.QtCharts import QChart, QChartView, QScatterSeries, QValueAxis
from PySide6.QtCore import QPointF, QRectF
from PySide6.QtGui import QPainter

class MainWindow(QMainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()
        self.ui = Ui_Form()
        self.ui.setupUi(self)

        # 初始化切割电机的图表和数据系列
        self.cut_motor_series = QScatterSeries()
        self.cut_motor_series.setMarkerSize(8)  # 增大点的大小使其更容易看见
        self.cut_motor_series.setName("编码器数据")
        
        # 创建图表
        self.cut_motor_chart = QChart()
        self.cut_motor_chart.addSeries(self.cut_motor_series)
        self.cut_motor_chart.setTitle("切割电机位置")
        self.cut_motor_chart.setAnimationOptions(QChart.SeriesAnimations)  # 添加动画效果
        
        # 设置坐标轴
        x_axis = QValueAxis()
        x_axis.setTitleText("编码器脉冲")
        x_axis.setRange(-50000, 50000)
        x_axis.setTickCount(11)
        x_axis.setLabelFormat("%d")  # 使用整数格式
        
        y_axis = QValueAxis()
        y_axis.setTitleText("扭矩(N.m)")
        y_axis.setRange(0, 100)
        y_axis.setTickCount(11)
        y_axis.setLabelFormat("%.1f")  # 显示一位小数
        
        # 添加坐标轴到图表
        self.cut_motor_chart.addAxis(x_axis, Qt.AlignBottom)
        self.cut_motor_chart.addAxis(y_axis, Qt.AlignLeft)
        self.cut_motor_series.attachAxis(x_axis)
        self.cut_motor_series.attachAxis(y_axis)
        
        # 设置图表外观
        self.cut_motor_chart.legend().setVisible(True)
        self.cut_motor_chart.legend().setAlignment(Qt.AlignBottom)
        self.cut_motor_chart.setBackgroundVisible(False)  # 设置背景透明
        self.cut_motor_chart.setMargins(QMargins(10, 10, 10, 10))  # 设置边距
        
        # 设置到 QChartview_cut_motor
        self.ui.QChartview_cut_motor.setChart(self.cut_motor_chart)
        self.ui.QChartview_cut_motor.setRenderHint(QPainter.Antialiasing)  # 抗锯齿
        
        # 初始化时间和数据点列表
        self.start_time = time.time()
        self.data_points = []

        # 用于记录时间
        self.start_time = time.time()
        self.data_points = []  # 存储数据点

        # 初始化通信和控制器对象
        self.motor_comm = None  # 电机通信对象
        self.motor_controller = None  # 电机控制器对象
        self.proximity_comm = None  # 接近开关通信对象
        self.proximity_controller = None  # 接近开关控制器对象
        self.penetration_motor_comm = None  # 贯入电机通信对象
        self.penetration_motor_controller = None  # 贯入电机控制器对象
        self.sensor_comm = None  # 传感器通信对象

        # 数获取定时器
        self.data_timer = QTimer(self)
        # self.data_timer.timeout.connect(self.get_encoder_data)
        self.data_timer.timeout.connect(self.update_cut_motor_plot)

        # 连接按钮点击事件到相应的方法
        self.ui.pushButton_open_com_motor.clicked.connect(self.open_motor_and_penetration_connection)  # 打开电机和贯入电机串口连接
        self.ui.pushButton_close_com_motor.clicked.connect(self.close_motor_and_penetration_connection)  # 关闭电机和贯入电机串口连接
        self.ui.pushButton_open_com_proximity_switch.clicked.connect(self.open_proximity_connection)  # 打开接近开关串口连接
        self.ui.pushButton_close_com_proximity_switch.clicked.connect(self.close_proximity_connection)  # 关闭接近开关串口连接
        self.ui.pushButton_clockwise_distance_cut_motor.clicked.connect(self.clockwise_distance_cut_motor)  # 顺时针旋转到指定距离
        self.ui.pushButton_anticlockwise_distance_cut_motor.clicked.connect(self.anticlockwise_distance_cut_motor)  # 逆时针旋转到指定距离
        self.ui.pushButton_stop_cut_motor.clicked.connect(self.stop_cut_motor)  # 停止电机
        self.ui.pushButton_clockwise_sustain_cut_motor.clicked.connect(self.clockwise_sustain_cut_motor)  # 顺时针持续旋转
        self.ui.pushButton_anticlockwise_sustain_cut_motor.clicked.connect(self.anticlockwise_sustain_cut_motor)  # 逆时针持续旋
        self.ui.pushButton_stop_penetration_motor.clicked.connect(self.stop_penetration_motor)  # 停止贯入电机
        self.ui.pushButton_up_sustain_penetration_motor.clicked.connect(self.up_sustain_penetration_motor)  # 持续上升贯入电机
        self.ui.pushButton_anticlockwise_sustain_penetration_motor.clicked.connect(self.anticlockwise_sustain_penetration_motor)  # 逆时针持续旋转贯入电机
        self.ui.pushButton_up_distance_penetration_motor.clicked.connect(self.up_distance_penetration_motor)  # 上升到指定距离
        self.ui.pushButton_down_distance_penetration_motor.clicked.connect(self.down_distance_penetration_motor)  # 下降到指定距离
        self.ui.pushButton_get_data_penetration_motor.clicked.connect(self.start_getting_penetration_encoder_data)  # 开始获取贯入电机编码器数据
        self.ui.pushButton_stop_data_penetration_motor.clicked.connect(self.stop_getting_penetration_encoder_data)  # 停止获取贯入电机编码器数据
        self.ui.pushButton_reset_penetration_motor.clicked.connect(self.reset_penetration_motor)  # 重置贯入电机
        self.ui.pushButton_reset_data_penetration_motor.clicked.connect(self.reset_data_penetration_motor)
        self.ui.pushButton_reset_data_cut_motor.clicked.connect(self.reset_data_cut_motor)

        # 连接刷新按钮点击事件到刷新方法
        self.ui.pushButton_refresh_com.clicked.connect(self.refresh_serial_ports)  # 刷新串口信息

        # 连接打开所有串口按钮点击事件到相应的方法
        self.ui.pushButton_open_com_all.clicked.connect(self.open_all_connections)  # 打开所有串口连接

        # 连接获取数据按钮点击事件到相应的方法
        self.ui.pushButton_get_data_cut_motor.clicked.connect(self.start_getting_encoder_data)  # 开始获取编码器数据
        self.ui.pushButton_stop_data_cut_motor.clicked.connect(self.stop_getting_encoder_data)  # 停止获取编码器数据

        # 填充串口列表
        self.populate_serial_ports()

    def closeEvent(self, event):
        """界面关闭时自动关闭已连接的串口"""
        self.close_motor_and_penetration_connection()  # 关闭电机和贯入电机串口连接
        self.close_proximity_connection()  # 关闭接近开关串口连接
        event.accept()  # 接受关闭事件

    def refresh_serial_ports(self):
        """刷新串口信息"""
        self.ui.comboBox_motor.clear()  # 清空电机串口下拉列表
        self.ui.comboBox_proximity_switch.clear()  # 清空接近开关串口下拉列表
        self.ui.comboBox_2_sensor.clear()  # 清空传感器串口下拉列表
        self.populate_serial_ports()  # 重新填充串口信息

    def populate_serial_ports(self):
        """填充可用的串口到下拉列表"""
        ports = serial.tools.list_ports.comports()  # 获取可用串口
        for port in ports:
            port_info = f"{port.device} - {port.description}"  # 组合串口信息
            self.ui.comboBox_motor.addItem(port_info)  # 添加到电机串口下拉列表
            self.ui.comboBox_proximity_switch.addItem(port_info)  # 添加到接近开关串口下拉列表
            self.ui.comboBox_2_sensor.addItem(port_info)  # 添加到传感器串口下拉列表
            if 'SERIAL-B' in port.description:
                self.ui.comboBox_motor.setCurrentText(port_info)  # 默认选择电机串口
            if 'SERIAL-A' in port.description:
                self.ui.comboBox_proximity_switch.setCurrentText(port_info)  # 默认选择接近开关串口
            if 'SERIAL-D' in port.description:
                self.ui.comboBox_2_sensor.setCurrentText(port_info)  # 默认选择传感器串口

    def open_motor_and_penetration_connection(self):
        """打开电机和贯入电机的串口连接"""
        self.open_cut_motor_connection()
        self.open_penetration_motor_connection()

    def open_cut_motor_connection(self):
        """打开电机的串口连接"""
        selected_port_info = self.ui.comboBox_motor.currentText()  # 获取选的串口息
        selected_port = selected_port_info.split(' - ')[0]  # 提取串口号
        try:
            self.motor_comm = CutMotorCommunication(selected_port)  # 创建电机通信对象
            self.motor_controller = CutMotorController(self.motor_comm)  # 创建电机控制器对象
            self.ui.label_message.setText(f"成功连接到 {selected_port}")  # 显示成功信息
        except Exception as e:
            self.ui.label_message.setText(f"无法连接到 {selected_port}: {str(e)}")  # 显示错误信息

    def close_cut_motor_connection(self):
        """关闭电机的串口连接"""
        if self.motor_comm:
            try:
                self.motor_comm.instrument.serial.close()  # 关闭串口连接
                self.motor_comm = None  # 清空通信对象
                self.motor_controller = None  # 清空控制器对象
                self.ui.label_message.setText("电机串口连接已断开")  # 显示断开信息
            except Exception as e:
                self.ui.label_message.setText(f"无法断开连接: {str(e)}")  # 显示错误信息

    def open_proximity_connection(self):
        """打开接近开关的串口连接"""
        selected_port_info = self.ui.comboBox_proximity_switch.currentText()  # 获取选中的串口信息
        selected_port = selected_port_info.split(' - ')[0]  # 提取串口号
        try:
            self.proximity_comm = ProximitySwitchCommunication(selected_port)  # 创建接近开关通信对象
            self.proximity_controller = ProximitySwitchController(self.proximity_comm)  # 创建接近开关控制器对象
            self.ui.label_message.setText(f"成功连接到 {selected_port}")  # 显示成功信息

            # 启动一个线程来监控开关状态
            self.proximity_monitor_thread = threading.Thread(target=self.monitor_proximity_switch)
            self.proximity_monitor_thread.start()
        except Exception as e:
            self.ui.label_message.setText(f"无法连接到 {selected_port}: {str(e)}")  # 显示错误信息

    def close_proximity_connection(self):
        """关闭接近开关的串口连接"""
        if self.proximity_comm:
            try:
                self.proximity_comm.instrument.serial.close()  # 关闭串口连接
                self.proximity_comm = None  # 清空通信对象
                self.proximity_controller = None  # 清空控制器对象
                self.ui.label_message.setText("接近开关串口连接已断开")  # 显示断开信息
            except Exception as e:
                self.ui.label_message.setText(f"无法断开连接: {str(e)}")  # 显示错误信息

    def open_penetration_motor_connection(self):
        """打开贯入电机的串口连接"""
        selected_port_info = self.ui.comboBox_motor.currentText()  # 获取选中的串口信息
        selected_port = selected_port_info.split(' - ')[0]  # 提取串口号
        try:
            self.penetration_motor_comm = PenetrationMotorCommunication(selected_port)  # 创建贯入电机通信对象
            self.penetration_motor_controller = PenetrationMotorController(self.penetration_motor_comm)  # 创建贯入电机控制器对象
            self.ui.label_message.setText(f"成功连接到 {selected_port}")  # 显示成功信息
        except Exception as e:
            self.ui.label_message.setText(f"无法连接到 {selected_port}: {str(e)}")  # 显示错误信息

    def close_penetration_motor_connection(self):
        """关闭贯入电机的串口连接"""
        if self.penetration_motor_comm:
            try:
                self.penetration_motor_comm.instrument.serial.close()  # 关闭串口连接
                self.penetration_motor_comm = None  # 清空通信对象
                self.penetration_motor_controller = None  # 清空控制器对象
                self.ui.label_message.setText("贯入电机串口连接已断开")  # 显示断开信息
            except Exception as e:
                self.ui.label_message.setText(f"无法断开连接: {str(e)}")  # 显示错误信息

    def monitor_proximity_switch(self):
        """监控接近开关状态"""
        try:
            while self.proximity_controller:
                status = self.proximity_controller.get_switch_status()
                if status is not None:
                    self.ui.label_message_proximity_switch.setText(f"开关量输入状态: {status}")
                time.sleep(0.1)
        except Exception as e:
            self.ui.label_message_proximity_switch.setText(f"监控失败: {str(e)}")

    def open_all_connections(self):
        """打开所有选中的串口连接"""
        self.open_motor_and_penetration_connection()  # 打开电机和贯入电机串口连接
        self.open_proximity_connection()  # 打开接近开关串口连接
        self.open_sensor_connection()  # 打开传感器串口连接

    def open_sensor_connection(self):
        """打开传感器的串口连接"""
        selected_port_info = self.ui.comboBox_2_sensor.currentText()  # 获取选中的串口信息
        selected_port = selected_port_info.split(' - ')[0]  # 提取串口号
        try:
            self.sensor_comm = CutMotorCommunication(selected_port)  # 创建传感器通信对象
            self.ui.label_message.setText(f"成功连接到 {selected_port}")  # 显示成功信息
        except Exception as e:
            self.ui.label_message.setText(f"无法连接到 {selected_port}: {str(e)}")  # 显示错误信息

    def clockwise_distance_cut_motor(self):
        """设置电机顺时针旋转到指定距离"""
        try:
            if self.motor_controller:
                self.motor_controller.set_servo_mode(2)  # 设置伺服模式为2
                speed_value = float(self.ui.lineEdit_speed_cut_motor.text())  # 获取速度值
                distance_value = float(self.ui.lineEdit_distance_cut_motor.text())  # 获取距离值

                speed_rpm = speed_value * 850 / 6  # 计算速度RPM
                target_position = int(distance_value * 37400 / 360)  # 计算目标位置
                position_type = 1  # 相对位置

                self.motor_controller.set_position_control(speed_rpm, position_type, target_position)  # 设置位置控制
                self.ui.label_message.setText("电机已设置为顺时针旋转到指定距离")  # 显示成功信息

                # 重置数据并开始绘图
                self.reset_data_cut_motor()
                if not self.data_timer.isActive():
                    self.data_timer.timeout.connect(self.update_cut_motor_plot)
                    self.data_timer.start(100)
            else:
                self.ui.label_message.setText("电机控制器未连接")  # 显示未连接信息
        except ValueError as e:
            self.ui.label_message.setText(f"无效的输入: {str(e)}")  # 显示输入错误信息
        except Exception as e:
            self.ui.label_message.setText(f"无法设置电机: {str(e)}")  # 显示设置错误信息

    def anticlockwise_distance_cut_motor(self):
        """置电机逆时针旋转到指定距离"""
        try:
            if self.motor_controller:
                self.motor_controller.set_servo_mode(2)  # 设置伺服模式为2
                speed_value = float(self.ui.lineEdit_speed_cut_motor.text())  # 获取速度值
                distance_value = float(self.ui.lineEdit_distance_cut_motor.text())  # 获取距离值

                speed_rpm = speed_value * 850 / 6  # 计算速度RPM
                target_position = -int(distance_value * 37400 / 360)  # 计算目标位置，设为负数
                position_type = 1  # 相对位置

                self.motor_controller.set_position_control(speed_rpm, position_type, target_position)  # 置位置控制
                self.ui.label_message.setText("电机已设置为逆针旋转到指定距离")  # 显示成功信息
            else:
                self.ui.label_message.setText("电机控制器未连接")  # 显示未连接信息
        except ValueError as e:
            self.ui.label_message.setText(f"无效的输入: {str(e)}")  # 显示输入错误信息
        except Exception as e:
            self.ui.label_message.setText(f"无法设置电机: {str(e)}")  # 显示设置错误信息

    def stop_cut_motor(self):
        """停止电机"""
        try:
            if self.motor_controller:
                self.motor_controller.set_speed(0)  # 设置速度为0，停止电机
                self.ui.label_message.setText("电机已停止")  # 显示停止信息
            else:
                self.ui.label_message.setText("电机控制器未连接")  # 显示未连接信息
        except Exception as e:
            self.ui.label_message.setText(f"无法停止电机: {str(e)}")  # 显示停止错误信息

    def clockwise_sustain_cut_motor(self):
        """设置电机顺时针持续旋转"""
        try:
            if self.motor_controller:
                speed_value = float(self.ui.lineEdit_speed_cut_motor.text())  # 获取速度值
                speed_rpm = speed_value * 850 / 6  # 计算速度RPM

                self.motor_controller.set_speed(speed_rpm)  # 设置电机速度
                self.ui.label_message.setText("电机已设置为顺时针持续旋转")  # 显示成功信息
            else:
                self.ui.label_message.setText("电机控制器未连接")  # 显示未连接信息
        except ValueError as e:
            self.ui.label_message.setText(f"无效的输入: {str(e)}")  # 显示输入错误信息
        except Exception as e:
            self.ui.label_message.setText(f"无法设置电机: {str(e)}")  # 显示设置错误信息

    def anticlockwise_sustain_cut_motor(self):
        """设置电机逆时针持续旋转"""
        try:
            if self.motor_controller:
                speed_value = float(self.ui.lineEdit_speed_cut_motor.text())  # 获取速度值
                speed_rpm = -speed_value * 850 / 6  # 计算速度RPM，设为负数以逆时针旋转

                self.motor_controller.set_speed(speed_rpm)  # 设置电机速度
                self.ui.label_message.setText("电机已设置为逆时针持续旋转")  # 显示成功信息
            else:
                self.ui.label_message.setText("电机控制器未连接")  # 显示未连接信息
        except ValueError as e:
            self.ui.label_message.setText(f"无效的输入: {str(e)}")  # 显示输入错误信息
        except Exception as e:
            self.ui.label_message.setText(f"无法设置电机: {str(e)}")  # 显示设置错误信息

    def start_getting_encoder_data(self):
        """开始获取编码器数据"""
        if not self.data_timer.isActive():
            self.data_timer.timeout.connect(self.get_encoder_data)  # Changed: connect to get_encoder_data instead of update_cut_motor_plot
            self.data_timer.start(100)  # 每100毫秒更新一次数据

    def stop_getting_encoder_data(self):
        """停止获取编码器数据"""
        if self.data_timer.isActive():
            self.data_timer.stop()

    def get_encoder_data(self):
        """获取编码器数据并显示"""
        try:
            if self.motor_controller:
                encoder_pulses = self.motor_controller.get_encoder_pulses()  # 获取编码器脉冲数据
                self.ui.label_message_cut_motor.setText(f"编码器脉冲数量: {encoder_pulses}")  # 显示编码器数据
                return encoder_pulses
            else:
                self.ui.label_message_cut_motor.setText("电机控制器未连接")  # 显示未连接信息
                return None
        except Exception as e:
            self.ui.label_message_cut_motor.setText(f"无法获取编码器数据: {str(e)}")  # 显示错误信息
            return None

    def close_motor_and_penetration_connection(self):
        """关闭电机和贯入电机的串口连接"""
        self.close_cut_motor_connection()
        self.close_penetration_motor_connection()

    def stop_penetration_motor(self):
        """停止贯入电机"""
        try:
            if self.penetration_motor_controller:
                self.penetration_motor_controller.set_speed(0)  # 设置速度为0，停止电机
                self.ui.label_message.setText("贯入电机已停止")  # 显示停止信息
            else:
                self.ui.label_message.setText("贯入电机控制器未连接")  # 显示未连接信息
        except Exception as e:
            self.ui.label_message.setText(f"无法停止贯入电机: {str(e)}")  # 显示停止错误信息

    def up_sustain_penetration_motor(self):
        """设置贯入电机持续上升"""
        threading.Thread(target=self._up_sustain_penetration_motor).start()

    def _up_sustain_penetration_motor(self):
        """设置贯入电机持续上升的实际操作"""
        try:
            if self.penetration_motor_controller:
                speed_value = float(self.ui.lineEdit_speed_penetration_motor.text())  # 获取速度值
                speed_rpm = speed_value * 1020  # 计算速度RPM

                self.penetration_motor_controller.set_speed(speed_rpm)  # 设置电机速度
                self.ui.label_message.setText("贯入电机已设置为持续上升")  # 显示成功信息

                # 监控接近开关状态
                while True:
                    switch_status = self.proximity_controller.get_switch_status()
                    if switch_status == [0]:  # 如果开关状态为0
                        self.penetration_motor_controller.set_speed(0)  # 立即停止电机
                        self.ui.label_message.setText("贯入电机因开关状态停止")  # 显示停止信息
                        break
                    time.sleep(0.1)  # 每100毫秒检查一次
            else:
                self.ui.label_message.setText("贯入电机控制器未连接")  # 显示未连接信息
        except ValueError as e:
            self.ui.label_message.setText(f"无效的输入: {str(e)}")  # 显示输入错误信息
        except Exception as e:
            self.ui.label_message.setText(f"无法设置贯入电机: {str(e)}")  # 显示设置错误信息

    def anticlockwise_sustain_penetration_motor(self):
        """设置贯入电机逆时针持续旋转"""
        try:
            if self.penetration_motor_controller:
                speed_value = float(self.ui.lineEdit_speed_penetration_motor.text())  # 获取速度值
                speed_rpm = -speed_value * 1020  # 计算速度RPM，设为负数以逆时针旋转

                self.penetration_motor_controller.set_speed(speed_rpm)  # 设置电机速度
                self.ui.label_message.setText("贯入电机已设置为逆时针持续旋转")  # 显示成功信息
            else:
                self.ui.label_message.setText("贯入电机控制器未连接")  # 显示未连接信息
        except ValueError as e:
            self.ui.label_message.setText(f"无效的输入: {str(e)}")  # 显示输入错误信息
        except Exception as e:
            self.ui.label_message.setText(f"无法设置贯入电机: {str(e)}")  # 显示设置错误信息

    def up_distance_penetration_motor(self):
        """设置贯入电机上升到指定距离的实际操作"""
        try:
            if self.penetration_motor_controller:
                self.penetration_motor_controller.set_servo_mode(2)  # 设置伺服模式为2
                speed_value = float(self.ui.lineEdit_speed_penetration_motor.text())  # 获取速度值
                distance_value = float(self.ui.lineEdit_distance_penetration_motor.text())  # 获取距离值

                speed_rpm = speed_value * 1020  # 计算速度RPM
                target_position = int(distance_value * 544)  # 计算目标位置
                position_type = 1  # 相对位置

                self.penetration_motor_controller.set_position_control(speed_rpm, position_type, target_position)  # 设置位置控制
                self.ui.label_message.setText("贯入电机已设置为上升到指定距离")  # 显示成功信息
            else:
                self.ui.label_message.setText("贯入电机控制器未连接")  # 显示未连接信息
        except ValueError as e:
            self.ui.label_message.setText(f"无效的输入: {str(e)}")  # 显示输入错误信息
        except Exception as e:
            self.ui.label_message.setText(f"无法设置贯入电机: {str(e)}")  # 显示设置错误信息

    def down_distance_penetration_motor(self):
        """设置贯入电机下降到指定距离"""
        try:
            if self.penetration_motor_controller:
                self.penetration_motor_controller.set_servo_mode(2)  # 设置伺服模式为2
                speed_value = float(self.ui.lineEdit_speed_penetration_motor.text())  # 获取速度值
                distance_value = float(self.ui.lineEdit_distance_penetration_motor.text())  # 获取距离值

                speed_rpm = -speed_value * 1020  # 计算速度RPM，设为负数以逆时针旋转
                target_position = -int(distance_value * 544)  # 计算目标位置，设为负数
                position_type = 1  # 相对位置

                self.penetration_motor_controller.set_position_control(speed_rpm, position_type, target_position)  # 设置位置控制
                self.ui.label_message.setText("贯入电机已设置为下降到指定距离")  # 显示成功信息
            else:
                self.ui.label_message.setText("贯入电机控制器未连接")  # 显示未连接信息
        except ValueError as e:
            self.ui.label_message.setText(f"无效的输入: {str(e)}")  # 显示输入错误信息
        except Exception as e:
            self.ui.label_message.setText(f"无法设置贯入电机: {str(e)}")  # 显示设置错误信息

    def start_getting_penetration_encoder_data(self):
        """开始获取贯入电机编码器数据"""
        if not self.data_timer.isActive():
            self.data_timer.timeout.connect(self.get_penetration_encoder_data)
            self.data_timer.start(100)  # 每100毫秒更新一次数据

    def stop_getting_penetration_encoder_data(self):
        """停止获取贯入电机编码器数据"""
        if self.data_timer.isActive():
            self.data_timer.stop()

    def get_penetration_encoder_data(self):
        """获取贯入电机编码器数据并显示"""
        try:
            if self.penetration_motor_controller:
                encoder_pulses = self.penetration_motor_controller.get_encoder_pulses()  # 获取编码器脉冲数据
                self.ui.label_message_penetration_motor.setText(f"编码器脉冲数量: {encoder_pulses}")  # 显示编码器数据
            else:
                self.ui.label_message_penetration_motor.setText("贯入电机控制器未连接")  # 显示未连接信息
        except Exception as e:
            self.ui.label_message_penetration_motor.setText(f"无法获取编码器数据: {str(e)}")  # 显示错误信息

    def reset_penetration_motor(self):
        """重置贯入电机"""
        threading.Thread(target=self._reset_penetration_motor).start()

    def _reset_penetration_motor(self):
        """重置贯入电机的实际操作"""
        try:
            if self.penetration_motor_controller:
                speed_rpm = 501  # 设置速度为501 RPM
                self.penetration_motor_controller.set_speed(speed_rpm)  # 设置电机速度
                self.ui.label_message.setText("贯入电机正在重置...")  # 显示重置信息

                # 监控接近开关状态
                while True:
                    switch_status = self.proximity_controller.get_switch_status()
                    print(f"当前开关状态: {switch_status}")  # 添加调试输出
                    if switch_status == [0]:  # 如果开关状态为0
                        self.penetration_motor_controller.set_speed(0)  # 立即停止电机
                        self.ui.label_message.setText("贯入电机重置完成")  # 显示完成信息
                        break
                    time.sleep(0.1)  # 每100毫秒检查一次
            else:
                self.ui.label_message.setText("贯入电机控制器未连接")  # 显示未连接信息
        except Exception as e:
            self.ui.label_message.setText(f"无法重置贯入电机: {str(e)}")  # 显示错误信息

    def reset_data_penetration_motor(self):
        """重置贯入电机数据"""
        try:
            if self.penetration_motor_controller:
                self.penetration_motor_controller.reset_data()
                self.ui.label_message.setText("贯入电机数据已重置")
            else:
                self.ui.label_message.setText("贯入电机控制器未连接")
        except Exception as e:
            self.ui.label_message.setText(f"无法重置贯入电机数据: {str(e)}")

    def reset_data_cut_motor(self):
        """重置切割电机的数据"""
        try:
            if self.motor_controller:  # Changed back to motor_controller
                self.motor_controller.reset_data()  # 重置电机控制器数据
            
            self.data_points = []  # 清空数据点列表
            self.cut_motor_series.clear()  # 清空图表系列
            self.start_time = time.time()  # 重置开始时间
            
            # 重置图表轴范围
            self.cut_motor_chart.axisX().setRange(-50000, 50000)
            self.cut_motor_chart.axisY().setRange(-50000, 50000)
            
            self.ui.label_message_cut_motor.setText("切割电机数据已重置")
            
            # 停止定时器
            if self.data_timer.isActive():
                self.data_timer.stop()
                # 断开所有连接
                try:
                    self.data_timer.timeout.disconnect()
                except:
                    pass
        except Exception as e:
            self.ui.label_message_cut_motor.setText(f"重置切割电机数据失败: {str(e)}")

    def start_plotting_cut_motor(self):
        """开始绘制电机编码器数据"""
        if not self.data_timer.isActive():
            self.data_timer.timeout.connect(self.update_cut_motor_plot)
            self.data_timer.start(100)  # 每100毫秒更新一次数据

    def update_cut_motor_plot(self):
        """更新切割电机编码器数据的绘图"""
        try:
            encoder_pulses = self.get_encoder_data()
            if encoder_pulses is not None:
                # 使用固定的 y 值 200
                x = float(encoder_pulses)
                y = float(encoder_pulses * 1.5)
                
                # 添加新的数据点
                self.data_points.append((x, y))
                
                # 清除现有数据点
                self.cut_motor_series.clear()
                
                # 添加所有数据点
                for point_x, point_y in self.data_points:
                    self.cut_motor_series.append(point_x, point_y)
                
                # 动态调整 X 和 Y 轴范围
                if self.data_points:
                    # X 轴范围调整
                    x_values = [p[0] for p in self.data_points]
                    x_min = min(x_values)
                    x_max = max(x_values)
                    x_margin = (x_max - x_min) * 0.1 if x_max != x_min else 1000
                    self.cut_motor_chart.axisX().setRange(x_min - x_margin, x_max + x_margin)
                    
                    # Y 轴范围调整
                    y_values = [p[1] for p in self.data_points]
                    y_min = min(y_values)
                    y_max = max(y_values)
                    y_margin = (y_max - y_min) * 0.1 if y_max != y_min else 1000
                    self.cut_motor_chart.axisY().setRange(y_min - y_margin, y_max + y_margin)
                
        except Exception as e:
            print(f"更新图表时出错: {str(e)}")
            self.ui.label_message_cut_motor.setText(f"更新图表失败: {str(e)}")

def main():
    app = QApplication([])
    window = MainWindow()
    window.show()
    app.exec()

if __name__ == "__main__":
    main()
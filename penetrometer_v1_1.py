from PySide6.QtWidgets import QApplication, QMainWindow, QGraphicsScene, QFileDialog
from gui_v1_english import Ui_Form
from cut_motor_control.communication import CutMotorCommunication
from cut_motor_control.controller import CutMotorController
from proximity_switch.communication import ProximitySwitchCommunication
from proximity_switch.controller import ProximitySwitchController
from penetration_motor_control.communication import PenetrationMotorCommunication
from penetration_motor_control.controller import PenetrationMotorController
from sensor_2.communication import SensorCommunication
from sensor_2.controller import SensorController
import serial.tools.list_ports
import threading
import time
from PySide6.QtCore import QTimer, Qt, QMargins
from PySide6.QtCharts import QChart, QChartView, QScatterSeries, QValueAxis
from PySide6.QtCore import QPointF, QRectF
from PySide6.QtGui import QPainter, QColor, QIcon
import pandas as pd
from datetime import datetime
import os

class MainWindow(QMainWindow):
    """
    主窗口类，负责管理GUI界面和所有控制功能
    
    属性:
        ui: GUI界面实例
        motor_controller: 剪切电机控制器
        penetration_motor_controller: 贯入电机控制器
        proximity_controller: 接近开关控制器
        cut_motor_data_points: 存储图表数据点的列表
        cut_motor_series: 剪切电机数据图表系列
        cut_motor_chart: 剪切电机数据图表
    """
    def __init__(self):
        """初始化主窗口及其组件"""
        super(MainWindow, self).__init__()
        self.ui = Ui_Form()
        self.ui.setupUi(self)

        # 初始化图表组件
        self.cut_motor_series = QScatterSeries()  # 创建散点图系列
        self.cut_motor_series.setName("Cut data points")  # 设置图表系列名称
        
        # 配置散点样式
        self.cut_motor_series.setMarkerSize(10)  # 设置点的大小
        self.cut_motor_series.setColor(QColor(0, 114, 189))  # 设置点的颜色
        
        # 创建并配置图表
        self.cut_motor_chart = QChart()
        self.cut_motor_chart.addSeries(self.cut_motor_series)
        self.cut_motor_chart.setTitle("Cut scatter plot")
        self.cut_motor_chart.setAnimationOptions(QChart.SeriesAnimations)
        
        # 配置X轴
        x_axis = QValueAxis()
        x_axis.setTitleText("Angle(°)")
        x_axis.setRange(-30, 30)
        x_axis.setTickCount(11)
        x_axis.setLabelFormat("%.1f")  # 显示一位小数
        
        # 配置Y轴
        y_axis = QValueAxis()
        y_axis.setTitleText("Torque(N.m)")
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
        self.cut_motor_data_points = []

        # 用于记录时间
        self.start_time = time.time()
        self.cut_motor_data_points = []  # 存储数据��

        # 初始化通信控制器对象
        self.cut_motor_comm = None  # 电机通信对象
        self.cut_motor_controller = None  # 电机控制器对象
        self.proximity_comm = None  # 接近开关通信对象
        self.proximity_controller = None  # 接近开关控制器对象
        self.penetration_motor_comm = None  # 贯入电机通信对象
        self.penetration_motor_controller = None  # 贯入电机控制器对象
        self.sensor_comm = None  # 传感器通信对象

        # 数获取定时器
        self.data_timer_cut = QTimer(self)
        # self.data_timer_cut.timeout.connect(self.get_cut_encoder_data)
        self.data_timer_cut.timeout.connect(self.update_cut_motor_plot)

        self.data_timer_penetration = QTimer(self)  # 用于贯入电机
        self.data_timer_penetration.timeout.connect(self.update_penetration_motor_plot)

        # 连接按钮点击事件到相应的方法
        self.ui.pushButton_open_com_motor.clicked.connect(self.open_motor_and_penetration_connection)  # 打开电机和贯入电机串口连接
        self.ui.pushButton_close_com_motor.clicked.connect(self.close_motor_and_penetration_connection)  # 关闭电机和贯入电机串口连接
        self.ui.pushButton_open_com_proximity_switch.clicked.connect(self.open_proximity_connection)  # 打开接近开���串口连接
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
        self.ui.pushButton_reset_data_penetration_motor.clicked.connect(self.reset_data_penetration_motor)  # 重置贯入电机数据
        self.ui.pushButton_reset_data_cut_motor.clicked.connect(self.reset_data_cut_motor)  # 重置剪切电机数据
        self.ui.pushButton_reset_figure_cut_motor.clicked.connect(self.reset_figure_cut_motor)  # 清空剪切电机图表
        self.ui.pushButton_open_com_sensor_2.clicked.connect(self.open_sensor_connection)  # 打开传感器串口连接
        self.ui.pushButton_close_com_sensor_2.clicked.connect(self.close_sensor_connection)  # 关闭传感器串口连接
        self.ui.pushButton_reset_figure_penetration_motor.clicked.connect(self.reset_figure_penetration_motor)
        self.ui.pushButton_reset_data_penetration_sensor.clicked.connect(self.reset_data_penetration_sensor)
        self.ui.pushButton_reset_data_cut_sensor.clicked.connect(self.reset_data_cut_sensor)

        # 连接刷新按钮点击事件到刷新方法
        self.ui.pushButton_refresh_com.clicked.connect(self.refresh_serial_ports)  # 刷新串口信息

        # 连打开所有串口按钮点击事件到相应的方法
        self.ui.pushButton_open_com_all.clicked.connect(self.open_all_connections)  # 打开所有串口连接

        # 连接取数据按钮点击事件到相的方法
        self.ui.pushButton_get_data_cut_motor.clicked.connect(self.start_getting_cut_encoder_data)  # 开始获取编码器数据
        self.ui.pushButton_stop_data_cut_motor.clicked.connect(self.stop_getting_cut_encoder_data)  # 停止获取编码器数据

        # 填充串口列表
        self.populate_serial_ports()

        # 连接文件夹选择和数据保存按钮
        self.ui.pushButton_select_folder_cut_motor.clicked.connect(self.select_folder_cut_motor)
        self.ui.pushButton_save_data_cut_motor.clicked.connect(self.save_data_cut_motor)
        
        self.ui.pushButton_select_folder_penetration_motor.clicked.connect(self.select_folder_penetration_motor)
        self.ui.pushButton_save_data_penetration_motor.clicked.connect(self.save_data_penetration_motor)

        # 初始化图表组件
        self.penetration_motor_series = QScatterSeries()  # 创建散点图系列
        self.penetration_motor_series.setName("Pressure data points")  # 设置图表系列名称

        # 配置散点样式
        self.penetration_motor_series.setMarkerSize(10)  # 设置点的大小
        self.penetration_motor_series.setColor(QColor(255, 0, 0))  # 设置点的颜色（红色）

        # 创建并配置图表
        self.penetration_motor_chart = QChart()
        self.penetration_motor_chart.addSeries(self.penetration_motor_series)
        self.penetration_motor_chart.setTitle("Penetration scatter plot")
        self.penetration_motor_chart.setAnimationOptions(QChart.SeriesAnimations)

        # 配置X轴
        x_axis_penetration = QValueAxis()
        x_axis_penetration.setTitleText("Distance(mm)")
        x_axis_penetration.setRange(-30, 30)  # 根据需要调整范围
        x_axis_penetration.setTickCount(11)
        x_axis_penetration.setLabelFormat("%d")  # 使用整数格式

        # 配置Y轴
        y_axis_penetration = QValueAxis()
        y_axis_penetration.setTitleText("Pressure(N)")
        y_axis_penetration.setRange(0, 100)  # 根据需要调整范围
        y_axis_penetration.setTickCount(11)
        y_axis_penetration.setLabelFormat("%.1f")  # 显示一位小数

        # 添加坐标轴到图表
        self.penetration_motor_chart.addAxis(x_axis_penetration, Qt.AlignBottom)
        self.penetration_motor_chart.addAxis(y_axis_penetration, Qt.AlignLeft)
        self.penetration_motor_series.attachAxis(x_axis_penetration)
        self.penetration_motor_series.attachAxis(y_axis_penetration)

        # 设置图表外观
        self.penetration_motor_chart.legend().setVisible(True)
        self.penetration_motor_chart.legend().setAlignment(Qt.AlignBottom)
        self.penetration_motor_chart.setBackgroundVisible(False)  # 设置背景透明
        self.penetration_motor_chart.setMargins(QMargins(10, 10, 10, 10))  # 设置边距

        # 设置到 QChartview_penetration_motor
        self.ui.QChartview_penetration_motor.setChart(self.penetration_motor_chart)
        self.ui.QChartview_penetration_motor.setRenderHint(QPainter.Antialiasing)  # 抗锯齿

        # 初始化时间和数据点列表
        self.start_time_penetration = time.time()
        self.penetration_motor_data_points = []  # 存储数据点

        # 初始化时间和数据点列表
        self.start_time_penetration = time.time()
        self.penetration_motor_data_points = []



    def closeEvent(self, event):
        """界面关闭时动关闭已连接的串口和线程"""
        # 关闭电机和贯入电机串口连接
        self.close_motor_and_penetration_connection()  
        # 关闭接近开关串口连接
        self.close_proximity_connection()  
        # 关闭传感器串口连接
        self.close_sensor_connection()  
        
        # 如果有监控线程，确保它们被安全关闭
        if hasattr(self, 'proximity_monitor_thread') and self.proximity_monitor_thread.is_alive():
            # 设置标志以停止监控线程
            self.proximity_monitor_thread.join()  # 等待线程结束

        # 关闭其他可能的线程（如果有）
        # 例如，如果您有其他线程，可以在这里添加关闭逻辑

        event.accept()  # 接受关闭事件

    def refresh_serial_ports(self):
        """刷新串口信息"""
        self.ui.comboBox_motor.clear()  # 清空电机串口下拉列表
        self.ui.comboBox_proximity_switch.clear()  # 清空接近开关串口下拉列表
        self.ui.comboBox_sensor_2.clear()  # 清空传感器串口下拉列表
        self.populate_serial_ports()  # 重新填充串口信息

    def populate_serial_ports(self):
        """填充可用的串口到下拉列表"""
        ports = serial.tools.list_ports.comports()  # 获取可用串口
        for port in ports:
            port_info = f"{port.device} - {port.description}"  # 组合串口信息
            self.ui.comboBox_motor.addItem(port_info)  # 添加到电机串口下拉列表
            self.ui.comboBox_proximity_switch.addItem(port_info)  # 添加到接近开关串口下拉列表
            self.ui.comboBox_sensor_2.addItem(port_info)  # 添加到传感器串口下拉列表
            if 'SERIAL-B' in port.description:
                self.ui.comboBox_motor.setCurrentText(port_info)  # 默认选择电机串口
            if 'SERIAL-A' in port.description:
                self.ui.comboBox_proximity_switch.setCurrentText(port_info)  # 默认选择接近开关串口
            if 'SERIAL-C' in port.description:
                self.ui.comboBox_sensor_2.setCurrentText(port_info)  # 默认选择传感器串口

    def open_motor_and_penetration_connection(self):
        """打开电机和贯入电机的串口连接"""
        self.open_cut_motor_connection()
        self.open_penetration_motor_connection()

    def open_cut_motor_connection(self):
        """打开电机的串口连接"""
        selected_port_info = self.ui.comboBox_motor.currentText()  # 获取选的串口息
        selected_port = selected_port_info.split(' - ')[0]  # 提取串口号
        try:
            self.cut_motor_comm = CutMotorCommunication(selected_port)  # 创建电机通信对象
            self.cut_motor_controller = CutMotorController(self.cut_motor_comm)  # 创建电机控制器对象
            self.ui.label_message.setText(f"成功连接到 {selected_port}")  # 显示成功信息
        except Exception as e:
            self.ui.label_message.setText(f"无法连接到 {selected_port}: {str(e)}")  # 显示错误信息

    def close_cut_motor_connection(self):
        """关闭电机的串口连接"""
        if self.cut_motor_comm:
            try:
                self.cut_motor_comm.instrument.serial.close()  # 关闭串口连接
                self.cut_motor_comm = None  # 清空通信对象
                self.cut_motor_controller = None  # 清空控制器对象
                self.ui.label_message.setText("电机串口连接已断开")  # 显示开信息
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

            # 启动一个线程来监控开状态
            self.proximity_monitor_thread = threading.Thread(target=self.monitor_proximity_switch)
            self.proximity_monitor_thread.start()
        except Exception as e:
            self.ui.label_message.setText(f"无法连接到 {selected_port}: {str(e)}")  # 显示错误信息

    def close_proximity_connection(self):
        """关闭接近开关的串口���接"""
        if self.proximity_comm:
            try:
                self.proximity_comm.instrument.serial.close()  # 关闭串口连接
                self.proximity_comm = None  # 清空通信对象
                self.proximity_controller = None  # 清控制器对象
                self.ui.label_message.setText("接近开关串口连接已断开")  # 显示断开信息
            except Exception as e:
                self.ui.label_message.setText(f"无法断开连接: {str(e)}")  # 显示错误息

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
                    self.ui.label_message_proximity_switch.setText(f"IO: {status}")
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
        selected_port_info = self.ui.comboBox_sensor_2.currentText()  # 获取选中的串口信息
        selected_port = selected_port_info.split(' - ')[0]  # 提取串口
        try:
            self.sensor_comm = SensorCommunication(selected_port)  # 创建传感器通信对象
            self.sensor_controller = SensorController(self.sensor_comm)  # 创建传感器控制器对象
            self.ui.label_message.setText(f"成功连接到 {selected_port}")  # 显示成功信息
        except Exception as e:
            self.ui.label_message.setText(f"无法连接到 {selected_port}: {str(e)}")  # 显示错误信息

    def close_sensor_connection(self):
        """关闭传感器的串口连接"""
        if self.sensor_comm:
            try:
                self.sensor_comm.instrument.serial.close()  # 关闭串口连接
                self.sensor_comm = None  # 清空通信对象
                self.sensor_controller = None  # 清控制器对象
                self.ui.label_message.setText("传感器串口连接已断开")  # 显示断开信息
            except Exception as e:
                self.ui.label_message.setText(f"无法断开连接: {str(e)}")  # 显示错误信息

    def clockwise_distance_cut_motor(self):
        """设置电机顺时针旋转到指定距离"""
        try:
            if self.cut_motor_controller:
                self.cut_motor_controller.set_servo_mode(2)  # 设置伺服模式为2
                speed_value = float(self.ui.lineEdit_speed_cut_motor.text())  # 获取速度值
                distance_value = float(self.ui.lineEdit_distance_cut_motor.text())  # 获取距离值

                speed_rpm = speed_value * 850 / 6  # 计算速度RPM
                target_position = int(distance_value * 37400 / 360)  # 计算目标位置
                position_type = 1  # 相对位置

                self.cut_motor_controller.set_position_control(speed_rpm, position_type, target_position)  # 设置位置控制
                self.ui.label_message.setText("电机已设置为顺时针旋转到指定距离")  # 显示成功信息

                if not self.data_timer_cut.isActive():
                    self.data_timer_cut.timeout.connect(self.update_cut_motor_plot)
                    self.data_timer_cut.start(100)
            else:
                self.ui.label_message.setText("电机控制器未连接")  # 显示未连接信息
        except ValueError as e:
            self.ui.label_message.setText(f"无效的输入: {str(e)}")  # 显示输入错误信息
        except Exception as e:
            self.ui.label_message.setText(f"无法设置电机: {str(e)}")  # 显示设置错误信息

    def anticlockwise_distance_cut_motor(self):
        """置电机逆时针旋转到指定距离"""
        try:
            if self.cut_motor_controller:
                self.cut_motor_controller.set_servo_mode(2)  # 设置伺服模式为2
                speed_value = float(self.ui.lineEdit_speed_cut_motor.text())  # 获取度值
                distance_value = float(self.ui.lineEdit_distance_cut_motor.text())  # 获取距离值

                speed_rpm = speed_value * 850 / 6  # 计算速度RPM
                target_position = -int(distance_value * 37400 / 360)  # 计算目标位置，设为负数
                position_type = 1  # 相对位置

                self.cut_motor_controller.set_position_control(speed_rpm, position_type, target_position)  # 置位置控制
                self.ui.label_message.setText("电机已设置为逆针旋转到指定距离")  # 显示成功信息

                if not self.data_timer_cut.isActive():
                    self.data_timer_cut.timeout.connect(self.update_cut_motor_plot)
                    self.data_timer_cut.start(100)
            else:
                self.ui.label_message.setText("电机控制器未连接")  # 显示未连接信息
        except ValueError as e:
            self.ui.label_message.setText(f"无效的输入: {str(e)}")  # 显示输入错误信息
        except Exception as e:
            self.ui.label_message.setText(f"无法设置电机: {str(e)}")  # 显示设置错误信息

    def stop_cut_motor(self):
        """停止电机"""
        try:
            if self.cut_motor_controller:
                self.cut_motor_controller.set_speed(0)  # 设置速度为0，停止电机
                self.ui.label_message.setText("电机已停止")  # 显示停止信息
            else:
                self.ui.label_message.setText("电机控制器未连接")  # 显示未连接信息
        except Exception as e:
            self.ui.label_message.setText(f"无法停止电机: {str(e)}")  # 显示停止错误信息

    def clockwise_sustain_cut_motor(self):
        """设置电机顺时针持续旋转"""
        try:
            if self.cut_motor_controller:
                speed_value = float(self.ui.lineEdit_speed_cut_motor.text())  # 获取速度值
                speed_rpm = speed_value * 850 / 6  # 计算速度RPM

                self.cut_motor_controller.set_speed(speed_rpm)  # 设置电机速度
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
            if self.cut_motor_controller:
                speed_value = float(self.ui.lineEdit_speed_cut_motor.text())  # 获取速度值
                speed_rpm = -speed_value * 850 / 6  # 计算速度RPM，设为负数以逆时针旋转

                self.cut_motor_controller.set_speed(speed_rpm)  # 设置电机速度
                self.ui.label_message.setText("电机已设置为逆时针持续旋转")  # 显示成功信息
            else:
                self.ui.label_message.setText("电机控制器未连接")  # 显示未连接信息
        except ValueError as e:
            self.ui.label_message.setText(f"无效的输入: {str(e)}")  # 显示输入错误信息
        except Exception as e:
            self.ui.label_message.setText(f"无法设置电机: {str(e)}")  # 显示设置错误信息

    def start_getting_cut_encoder_data(self):
        """开始获取剪切电机编码器数据"""
        if not self.data_timer_cut.isActive():
            self.data_timer_cut.timeout.connect(self.get_cut_encoder_data)  # 连接到 get_cut_encoder_data
            self.data_timer_cut.start(100)  # 每50毫秒更新一次数据，尽可能快地获取数据

    def stop_getting_cut_encoder_data(self):
        """停止获取编码器数据"""
        if self.data_timer_cut.isActive():
            self.data_timer_cut.stop()

    def get_cut_encoder_data(self):
        """获取编码器数据并显示"""
        try:
            if self.cut_motor_controller:
                encoder_pulses = self.cut_motor_controller.get_encoder_pulses()  # 获取编码器脉冲数据
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
            self.ui.label_message.setText(f"无法停止贯入电机: {str(e)}")  # 显示停止误信息

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
                self.ui.label_message.setText("贯入电机已置为持上升")  # 显示成功信息

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

                if not self.data_timer_penetration.isActive():
                    self.data_timer_penetration.timeout.connect(self.update_penetration_motor_plot)
                    self.data_timer_penetration.start(100)
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

                if not self.data_timer_penetration.isActive():
                    self.data_timer_penetration.timeout.connect(self.update_penetration_motor_plot)
                    self.data_timer_penetration.start(100)
            else:
                self.ui.label_message.setText("贯入电机控制器未连接")  # 显示未连接信息
        except ValueError as e:
            self.ui.label_message.setText(f"无效的输入: {str(e)}")  # 显示输入错误信息
        except Exception as e:
            self.ui.label_message.setText(f"无法设置贯入电机: {str(e)}")  # 显示设置错误信息

    def start_getting_penetration_encoder_data(self):
        """开获取贯入电机编码器数据"""
        if not self.data_timer_penetration.isActive():
            self.data_timer_penetration.timeout.connect(self.get_penetration_encoder_data)
            self.data_timer_penetration.start(100)  # 每100毫秒更新一次数据

    def stop_getting_penetration_encoder_data(self):
        """停止获取贯入电机编码器数据"""
        if self.data_timer_penetration.isActive():
            self.data_timer_penetration.stop()

    def get_penetration_encoder_data(self):
        """获取贯入电机的编码器数据"""
        try:
            # 假设这里是获取编码器数据的逻辑
            encoder_data = self.penetration_motor_controller.get_encoder_pulses()  # 从控制器获取编码器脉冲数据
                        
            return encoder_data  # 返回获取的编码器数据
        except Exception as e:
            print(f"获取编码器数据时出错: {str(e)}")  # 打印错误信息
            return None  # 返回None以指示出错

    def reset_penetration_motor(self):
        """归零贯入电机"""
        threading.Thread(target=self._reset_penetration_motor).start()

    def _reset_penetration_motor(self):
        """归零贯入电机的实际操作"""
        try:
            if self.penetration_motor_controller:
                speed_rpm = 501  # 设置速度为501 RPM
                self.penetration_motor_controller.set_speed(speed_rpm)  # 设置电机速度
                self.ui.label_message.setText("贯入电机正在归零...")  # 显示重置信息

                # 监控接近开关状态
                while True:
                    switch_status = self.proximity_controller.get_switch_status()
                    print(f"当前开关状态: {switch_status}")  # 添加调试输出
                    if switch_status == [0]:  # 如果开关状态为0
                        self.penetration_motor_controller.set_speed(0)  # 立即停止电机
                        self.ui.label_message.setText("贯入电机归零完成")  # 显示完成信息
                        break
                    time.sleep(0.1)  # 每100毫秒检查一次
            else:
                self.ui.label_message.setText("贯入电机控制器未连接")  # 显示未连接信息
        except Exception as e:
            self.ui.label_message.setText(f"无法归零贯入电机: {str(e)}")  # 显示错误信息

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
        """
        重置剪切电机的数据
        
        功能:
            - 重置电机控制器数据
        """
        try:
            if self.cut_motor_controller:
                self.cut_motor_controller.reset_data()  # 发送重置数据的命令
            
            self.ui.label_message_cut_motor.setText("剪切电机数据已重置")  # 更新状态信息
        except Exception as e:
            self.ui.label_message_cut_motor.setText(f"重置剪切电机数据失败: {str(e)}")

    def start_plotting_cut_motor(self):
        """开始绘制剪切电机数据"""
        if not self.data_timer_cut.isActive():
            self.data_timer_cut.timeout.connect(self.update_cut_motor_plot)  # 连接到更新函数
            self.data_timer_cut.start(100)  # 每100毫秒更新一次数据

    def start_plotting_penetration_motor(self):
        """开始绘制贯入电机数据"""
        if not self.data_timer_penetration.isActive():
            self.data_timer_penetration.timeout.connect(self.update_penetration_motor_plot)  # 连接到更新函数
            self.data_timer_penetration.start(100)  # 每100毫秒更新一次数据

    def update_cut_motor_plot(self):
        """
        更新剪切电机编码器数据的实时散点图显示
        
        功能:
            - 获取最新的编码器据
            - 更新数据点集合（包含位置、编码器数据和时间戳）
            - 重绘散点图
            - 动态调整坐标轴范围
        """
        try:
            encoder_pulses = self.get_cut_encoder_data()
            pulling_pressure, torque = self.get_sensor_data()  # 使用相同的方式获取传感器数据
            if encoder_pulses is not None:
                # 计算数据点坐标
                x = float(encoder_pulses * 360 / 37400)  # 编码器脉冲作为X轴数据
                y = float(torque)  # 使用torque值作为Y值
                current_time = (time.time() - self.start_time) * 1000  # 计算当前时间戳（毫秒）
                
                # 更新数据集合，现在每个点存储为三元组 (x, y, timestamp)
                self.cut_motor_data_points.append((x, y, current_time))
                
                # 更新散点图（只显示x和y）
                self.cut_motor_series.clear()  # 确保在添加新点之前清空系列
                for point_x, point_y, _ in self.cut_motor_data_points:
                    self.cut_motor_series.append(point_x, point_y)
                
                # 更新坐标轴范围
                if self.cut_motor_data_points:
                    # 计算并设置X轴范围
                    x_values = [p[0] for p in self.cut_motor_data_points]
                    x_min, x_max = min(x_values), max(x_values)
                    x_margin = (x_max - x_min) * 0.1 if x_max != x_min else 1000

                    # 获取X轴和Y轴
                    axes = self.cut_motor_chart.axes()  # 获取所有轴
                    x_axis = next((axis for axis in axes if isinstance(axis, QValueAxis) and axis.orientation() == Qt.Horizontal), None)
                    y_axis = next((axis for axis in axes if isinstance(axis, QValueAxis) and axis.orientation() == Qt.Vertical), None)

                    # 确保轴存在并设置范围
                    if x_axis is not None and y_axis is not None:  # 确保轴存在
                        x_axis.setRange(x_min - x_margin, x_max + x_margin)  # 设置X轴范围

                    # 计算并设置Y轴范围
                    y_values = [p[1] for p in self.cut_motor_data_points]
                    y_min, y_max = min(y_values), max(y_values)
                    y_margin = (y_max - y_min) * 0.1 if y_max != y_min else 1000

                    if y_axis is not None:  # 确保Y轴存在
                        y_axis.setRange(y_min - y_margin, y_max + y_margin)  # 设置Y轴范围
            
        except Exception as e:
            print(f"更新图表时出错: {str(e)}")
            self.ui.label_message_cut_motor.setText(f"更新图表失败: {str(e)}")

    def update_penetration_motor_plot(self):
        """
        更新贯入电机编码器数据的实时散点图显示
        
        功能:
            - 获取最新的编码器数据
            - 更新数据点集合（包含位置、编码器数据和时间戳）
            - 重绘散点图
            - 动态调整坐标轴范围
        """
        try:
            # 获取贯入电机的编码器脉冲数据
            encoder_pulses = self.get_penetration_encoder_data()  
            # 获取传感器数据，包括拉压力和扭矩
            pulling_pressure, torque = self.get_sensor_data()  
            
            if encoder_pulses is not None and pulling_pressure is not None:  # 确保数据有效
                # 计算数据点坐标
                x = float(encoder_pulses / 544)  # 下降距离（）作为X轴数据
                y = float(pulling_pressure)  # 拉压力作为Y轴数据
                # 计算当前时间戳（毫秒）
                current_time = (time.time() - self.start_time_penetration) * 1000  
                
                # 更新数据集合，现在每个点存储为三元组 (x, y, timestamp)
                self.penetration_motor_data_points.append((x, y, current_time))
                
                # 更新散点图（只显示x和y）
                self.penetration_motor_series.clear()  # 确保在添加新点之前清空系列
                for point_x, point_y, _ in self.penetration_motor_data_points:
                    self.penetration_motor_series.append(point_x, point_y)  # 添加数据点到系列

                # 更新坐标轴范围
                if self.penetration_motor_data_points:
                    # 计算并设置X轴范围
                    x_values = [p[0] for p in self.penetration_motor_data_points]  # 获取所有X值
                    x_min, x_max = min(x_values), max(x_values)  # 计算X轴的最小值和最大值
                    x_margin = (x_max - x_min) * 0.1 if x_max != x_min else 1000  # 设置X轴的边距

                    # 获取X轴和Y轴
                    axes = self.penetration_motor_chart.axes()  # 获取所有轴
                    x_axis = next((axis for axis in axes if isinstance(axis, QValueAxis) and axis.orientation() == Qt.Horizontal), None)  # 获取X轴
                    y_axis = next((axis for axis in axes if isinstance(axis, QValueAxis) and axis.orientation() == Qt.Vertical), None)  # 获取Y轴

                    # 确保轴存在并设置范围
                    if x_axis is not None and y_axis is not None:  # 确保轴存在
                        x_axis.setRange(x_min - x_margin, x_max + x_margin)  # 设置X轴范围

                    # 计算并设置Y轴范围
                    y_values = [p[1] for p in self.penetration_motor_data_points]  # 获取所有Y值
                    y_min, y_max = min(y_values), max(y_values)  # 计算Y轴的最小值和最大值
                    y_margin = (y_max - y_min) * 0.1 if y_max != y_min else 1000  # 设置Y轴的边距

                    if y_axis is not None:  # 确保Y轴存在
                        y_axis.setRange(y_min - y_margin, y_max + y_margin)  # 设置Y轴范围
            
        except Exception as e:
            # 捕获异常并打印错误信息
            print(f"更新图表时出错: {str(e)}")  
            # 在UI上显示错误信息
            self.ui.label_message_penetration_motor.setText(f"更新图表失败: {str(e)}")  

    def select_folder_cut_motor(self):
        """
        打开文件夹选择对话框，选择数据保存路径
        
        功能：
            - 打开文件选择窗口
            - 默认打开程序根目录
            - 将选择的路径显示在label中
        """
        try:
            # 获取程序根目录
            root_dir = os.path.dirname(os.path.abspath(__file__))
            
            # 打开文件夹选择对话框
            folder_path = QFileDialog.getExistingDirectory(
                self,
                "选择保存文件夹",
                root_dir,  # 默认打开根目录
                QFileDialog.ShowDirsOnly
            )
            
            # 如果用户选择了文件夹（没有点击取消
            if folder_path:
                # 更新label显示
                self.ui.label_folder_cut_motor.setText(folder_path)
                
        except Exception as e:
            self.ui.label_message.setText(f"选择文件夹失败: {str(e)}")

    def select_folder_penetration_motor(self):
        """
        打开文件夹选择对话框，选择数据保存路径
        
        功能：
            - 打开文件选择窗口
            - 默认打开程序根目录
            - 将选择的路径显示在label中
        """
        try:
            # 获取程序根目录
            root_dir = os.path.dirname(os.path.abspath(__file__))
            
            # 打开文件夹选择对话框
            folder_path = QFileDialog.getExistingDirectory(
                self,
                "选择保存文件夹",
                root_dir,  # 默认打开根目录
                QFileDialog.ShowDirsOnly
            )
            
            # 如果用户选择了文件夹（没有点击取消
            if folder_path:
                # 更新label显示
                self.ui.label_folder_penetration_motor.setText(folder_path)
                
        except Exception as e:
            self.ui.label_message.setText(f"选择文件夹失败: {str(e)}")

    def save_data_cut_motor(self):
        """
        保存剪切电机图表数据到Excel文件
        
        功能：
            - 获取图表中所有数据点
            - 包含位置、编码器数据和每个点的实际时间戳
            - 保存为xlsx格式
            - 文件名为"当前时间+用户输入名称"
        """
        try:
            # 检查是否有数据
            if not self.cut_motor_data_points:
                self.ui.label_message.setText("没有数据可保存")
                return
            
            # 获取保存路径
            save_folder = self.ui.label_folder_cut_motor.text()
            if not save_folder or not os.path.exists(save_folder):
                self.ui.label_message.setText("请先选择有效的保存路径")
                return
            
            # 获取用户输入的文件名
            user_filename = self.ui.lineEdit_file_name_cut_motor.text()
            if not user_filename:
                self.ui.label_message.setText("请输入文件名")
                return
            
            # 生成完整的文件名（cut_motor_当前时间+用户输入）
            current_time = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"cut_motor_{current_time}_{user_filename}.xlsx"
            full_path = os.path.join(save_folder, filename)
            
            # 准备数据，现在使用存储的时间戳
            data = {
                '剪切电机位置': [point[0] for point in self.cut_motor_data_points],
                '编码器数据': [point[1] for point in self.cut_motor_data_points],
                '时间戳(ms)': [point[2] for point in self.cut_motor_data_points]  # 使用存储的时间戳
            }
            
            # 创建DataFrame并保存
            df = pd.DataFrame(data)
            df.to_excel(full_path, index=False)
            
            self.ui.label_message.setText(f"数据已保存至: {filename}")
            
        except Exception as e:
            self.ui.label_message.setText(f"保存数据失败: {str(e)}")

    def save_data_penetration_motor(self):
        """
        保存剪切电机图表数据到Excel文件
        
        功能：
            - 获取图表中所有数据点
            - 包含位置、编码器数据和每个点的实际时间戳
            - 保存为xlsx格式
            - 文件名为"当前时间+用户输入名称"
        """
        try:
            # 检查是否有数据
            if not self.penetration_motor_data_points:
                self.ui.label_message.setText("没有数据可保存")
                return
            
            # 获取保存路径
            save_folder = self.ui.label_folder_penetration_motor.text()
            if not save_folder or not os.path.exists(save_folder):
                self.ui.label_message.setText("请先选择有效的保存路径")
                return
            
            # 获取用户输入的文件名
            user_filename = self.ui.lineEdit_file_name_penetration_motor_.text()
            if not user_filename:
                self.ui.label_message.setText("请输入文件名")
                return
            
            # 生成完整的文件名（penetration_motor_当前时间+用户输入）
            current_time = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"penetration_motor_{current_time}_{user_filename}.xlsx"
            full_path = os.path.join(save_folder, filename)
            
            # 准备数据，现在使用存储的时间戳
            data = {
                '贯入电机位置': [point[0] for point in self.penetration_motor_data_points],
                '拉压力': [point[1] for point in self.penetration_motor_data_points],
                '时间戳(ms)': [point[2] for point in self.penetration_motor_data_points]  # 使用存储的时间戳
            }
            
            # 创建DataFrame并保存
            df = pd.DataFrame(data)
            df.to_excel(full_path, index=False)
            
            self.ui.label_message.setText(f"数据已保存至: {filename}")
            
        except Exception as e:
            self.ui.label_message.setText(f"保存数据失败: {str(e)}")

    def reset_figure_cut_motor(self):
        """清空剪切电机图表"""
        self.cut_motor_series.clear()  # 清空散点图系列
        self.cut_motor_data_points.clear()  # 清空数据点列表
        # self.cut_motor_chart.axisX().setRange(0, 1)  # 重置X轴范围
        # self.cut_motor_chart.axisY().setRange(0, 1)  # 重置Y轴范围
        self.ui.label_message_cut_motor.setText("Reset figure")  # 更新状态信息

    def reset_figure_penetration_motor(self):
        """清空剪切电机图表"""
        self.penetration_motor_series.clear()  # 清空散点图系列
        self.penetration_motor_data_points.clear()  # 清空数据点列表
        # self.cut_motor_chart.axisX().setRange(0, 1)  # 重置X轴范围
        # self.cut_motor_chart.axisY().setRange(0, 1)  # 重置Y轴范围
        self.ui.label_message_penetration_motor.setText("Reset figure")  # 更新状态信息

    def start_getting_sensor_data(self):
        """开始获取传感器数据"""
        if not self.data_timer_cut.isActive():
            self.data_timer_cut.timeout.connect(self.get_sensor_data)  # 连接到 get_sensor_data
            self.data_timer_penetration.timeout.connect(self.get_sensor_data)
            self.data_timer_cut.start(100)  # 每50毫秒更新一次数据，尽可能快地获取数据

    def get_sensor_data(self):
        """获取传感器数据并显示"""
        try:
            pulling_pressure, torque = self.sensor_controller.monitor_2_sensor()  # 获取传感器数据
            self.ui.label_message.setText(f"Pressure: {pulling_pressure}, Torque: {torque}")  # 显示传感器数据
            return pulling_pressure, torque
        except Exception as e:
            self.ui.label_message.setText(f"无法获取传感器数据: {str(e)}")  # 显示错误信息
            return None, None

    def reset_data_penetration_sensor(self):
        """重置传感器的拉压力数据"""
        try:
            if self.sensor_controller:
                self.sensor_controller.reset_pulling_pressure()
                self.ui.label_message.setText("传感器拉压力数据已重置")
            else:
                self.ui.label_message.setText("传感器控制器未连接")
        except Exception as e:
            self.ui.label_message.setText(f"无法重置传感器拉压力数据: {str(e)}")

    def reset_data_cut_sensor(self):
        """重置传感器的扭矩数据"""
        try:
            if self.sensor_controller:
                self.sensor_controller.reset_torque()
                self.ui.label_message.setText("传感器扭矩数据已重置")
            else:
                self.ui.label_message.setText("传感器控制器未连接")
        except Exception as e:
            self.ui.label_message.setText(f"无法重置传感器扭矩数据: {str(e)}")

def main():
    app = QApplication([])
    window = MainWindow()
    window.show()
    window.setWindowTitle("TTL-V1.0")
    window.setWindowIcon(QIcon('jlu.png'))
    app.exec()

if __name__ == "__main__":
    main()
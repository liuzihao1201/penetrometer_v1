# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'gui_v1.ui'
##
## Created by: Qt User Interface Compiler version 6.7.2
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide6.QtCharts import QChartView
from PySide6.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect,
    QSize, QTime, QUrl, Qt)
from PySide6.QtGui import (QBrush, QColor, QConicalGradient, QCursor,
    QFont, QFontDatabase, QGradient, QIcon,
    QImage, QKeySequence, QLinearGradient, QPainter,
    QPalette, QPixmap, QRadialGradient, QTransform)
from PySide6.QtWidgets import (QApplication, QComboBox, QGroupBox, QLabel,
    QLineEdit, QPushButton, QSizePolicy, QWidget)

class Ui_Form(object):
    def setupUi(self, Form):
        if not Form.objectName():
            Form.setObjectName(u"Form")
        Form.resize(1382, 910)
        self.groupBox = QGroupBox(Form)
        self.groupBox.setObjectName(u"groupBox")
        self.groupBox.setGeometry(QRect(10, 10, 331, 81))
        self.pushButton_open_com_proximity_switch = QPushButton(self.groupBox)
        self.pushButton_open_com_proximity_switch.setObjectName(u"pushButton_open_com_proximity_switch")
        self.pushButton_open_com_proximity_switch.setGeometry(QRect(10, 50, 151, 23))
        self.pushButton_close_com_proximity_switch = QPushButton(self.groupBox)
        self.pushButton_close_com_proximity_switch.setObjectName(u"pushButton_close_com_proximity_switch")
        self.pushButton_close_com_proximity_switch.setGeometry(QRect(170, 50, 151, 23))
        self.comboBox_proximity_switch = QComboBox(self.groupBox)
        self.comboBox_proximity_switch.setObjectName(u"comboBox_proximity_switch")
        self.comboBox_proximity_switch.setGeometry(QRect(10, 20, 311, 22))
        self.groupBox_4 = QGroupBox(Form)
        self.groupBox_4.setObjectName(u"groupBox_4")
        self.groupBox_4.setGeometry(QRect(10, 90, 331, 81))
        self.pushButton_open_com_motor = QPushButton(self.groupBox_4)
        self.pushButton_open_com_motor.setObjectName(u"pushButton_open_com_motor")
        self.pushButton_open_com_motor.setGeometry(QRect(10, 50, 151, 23))
        self.pushButton_close_com_motor = QPushButton(self.groupBox_4)
        self.pushButton_close_com_motor.setObjectName(u"pushButton_close_com_motor")
        self.pushButton_close_com_motor.setGeometry(QRect(170, 50, 151, 23))
        self.comboBox_motor = QComboBox(self.groupBox_4)
        self.comboBox_motor.setObjectName(u"comboBox_motor")
        self.comboBox_motor.setGeometry(QRect(10, 20, 311, 22))
        self.groupBox_2_sensor = QGroupBox(Form)
        self.groupBox_2_sensor.setObjectName(u"groupBox_2_sensor")
        self.groupBox_2_sensor.setGeometry(QRect(10, 170, 331, 81))
        self.pushButton_open_com_2_sensor = QPushButton(self.groupBox_2_sensor)
        self.pushButton_open_com_2_sensor.setObjectName(u"pushButton_open_com_2_sensor")
        self.pushButton_open_com_2_sensor.setGeometry(QRect(10, 50, 151, 23))
        self.pushButton_close_com_2_sensor = QPushButton(self.groupBox_2_sensor)
        self.pushButton_close_com_2_sensor.setObjectName(u"pushButton_close_com_2_sensor")
        self.pushButton_close_com_2_sensor.setGeometry(QRect(170, 50, 151, 23))
        self.comboBox_2_sensor = QComboBox(self.groupBox_2_sensor)
        self.comboBox_2_sensor.setObjectName(u"comboBox_2_sensor")
        self.comboBox_2_sensor.setGeometry(QRect(10, 20, 311, 22))
        self.pushButton_open_com_all = QPushButton(Form)
        self.pushButton_open_com_all.setObjectName(u"pushButton_open_com_all")
        self.pushButton_open_com_all.setGeometry(QRect(120, 260, 101, 23))
        self.pushButton_8 = QPushButton(Form)
        self.pushButton_8.setObjectName(u"pushButton_8")
        self.pushButton_8.setGeometry(QRect(230, 260, 101, 23))
        self.label_message = QLabel(Form)
        self.label_message.setObjectName(u"label_message")
        self.label_message.setGeometry(QRect(410, 120, 791, 61))
        font = QFont()
        font.setPointSize(12)
        self.label_message.setFont(font)
        self.label_message.setStyleSheet(u"background-color: rgb(170, 255, 255);")
        self.label_message.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.groupBox_penetration_motor = QGroupBox(Form)
        self.groupBox_penetration_motor.setObjectName(u"groupBox_penetration_motor")
        self.groupBox_penetration_motor.setGeometry(QRect(10, 290, 331, 201))
        self.pushButton_reset_penetration_motor = QPushButton(self.groupBox_penetration_motor)
        self.pushButton_reset_penetration_motor.setObjectName(u"pushButton_reset_penetration_motor")
        self.pushButton_reset_penetration_motor.setGeometry(QRect(10, 20, 41, 23))
        self.pushButton_down_distance_penetration_motor = QPushButton(self.groupBox_penetration_motor)
        self.pushButton_down_distance_penetration_motor.setObjectName(u"pushButton_down_distance_penetration_motor")
        self.pushButton_down_distance_penetration_motor.setGeometry(QRect(110, 80, 71, 23))
        self.pushButton_up_distance_penetration_motor = QPushButton(self.groupBox_penetration_motor)
        self.pushButton_up_distance_penetration_motor.setObjectName(u"pushButton_up_distance_penetration_motor")
        self.pushButton_up_distance_penetration_motor.setGeometry(QRect(110, 50, 71, 23))
        self.lineEdit_speed_penetration_motor = QLineEdit(self.groupBox_penetration_motor)
        self.lineEdit_speed_penetration_motor.setObjectName(u"lineEdit_speed_penetration_motor")
        self.lineEdit_speed_penetration_motor.setGeometry(QRect(10, 50, 51, 21))
        self.lineEdit_speed_penetration_motor.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lineEdit_distance_penetration_motor = QLineEdit(self.groupBox_penetration_motor)
        self.lineEdit_distance_penetration_motor.setObjectName(u"lineEdit_distance_penetration_motor")
        self.lineEdit_distance_penetration_motor.setGeometry(QRect(10, 80, 51, 21))
        self.lineEdit_distance_penetration_motor.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.label_2 = QLabel(self.groupBox_penetration_motor)
        self.label_2.setObjectName(u"label_2")
        self.label_2.setGeometry(QRect(60, 50, 51, 21))
        font1 = QFont()
        font1.setPointSize(10)
        self.label_2.setFont(font1)
        self.label_2.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.label_3 = QLabel(self.groupBox_penetration_motor)
        self.label_3.setObjectName(u"label_3")
        self.label_3.setGeometry(QRect(60, 80, 51, 21))
        self.label_3.setFont(font1)
        self.label_3.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.pushButton_up_sustain_penetration_motor = QPushButton(self.groupBox_penetration_motor)
        self.pushButton_up_sustain_penetration_motor.setObjectName(u"pushButton_up_sustain_penetration_motor")
        self.pushButton_up_sustain_penetration_motor.setGeometry(QRect(240, 50, 81, 23))
        self.pushButton_anticlockwise_sustain_penetration_motor = QPushButton(self.groupBox_penetration_motor)
        self.pushButton_anticlockwise_sustain_penetration_motor.setObjectName(u"pushButton_anticlockwise_sustain_penetration_motor")
        self.pushButton_anticlockwise_sustain_penetration_motor.setGeometry(QRect(240, 80, 81, 23))
        self.pushButton_stop_data_penetration_motor = QPushButton(self.groupBox_penetration_motor)
        self.pushButton_stop_data_penetration_motor.setObjectName(u"pushButton_stop_data_penetration_motor")
        self.pushButton_stop_data_penetration_motor.setGeometry(QRect(190, 20, 61, 23))
        self.pushButton_get_data_penetration_motor = QPushButton(self.groupBox_penetration_motor)
        self.pushButton_get_data_penetration_motor.setObjectName(u"pushButton_get_data_penetration_motor")
        self.pushButton_get_data_penetration_motor.setGeometry(QRect(110, 20, 71, 23))
        self.pushButton_stop_penetration_motor = QPushButton(self.groupBox_penetration_motor)
        self.pushButton_stop_penetration_motor.setObjectName(u"pushButton_stop_penetration_motor")
        self.pushButton_stop_penetration_motor.setGeometry(QRect(60, 20, 41, 23))
        self.pushButton_stop_penetration_motor.setStyleSheet(u"color: rgb(255, 0, 0);\n"
"background-color: rgb(255, 255, 127);")
        self.pushButton_reset_data_penetration_motor = QPushButton(self.groupBox_penetration_motor)
        self.pushButton_reset_data_penetration_motor.setObjectName(u"pushButton_reset_data_penetration_motor")
        self.pushButton_reset_data_penetration_motor.setGeometry(QRect(260, 20, 61, 23))
        self.pushButton_reset_data_penetration_motor.setStyleSheet(u"color: rgb(255, 0, 0);\n"
"background-color: rgb(255, 255, 127);")
        self.pushButton_refresh_com = QPushButton(Form)
        self.pushButton_refresh_com.setObjectName(u"pushButton_refresh_com")
        self.pushButton_refresh_com.setGeometry(QRect(20, 260, 91, 23))
        self.groupBox_cut_motor = QGroupBox(Form)
        self.groupBox_cut_motor.setObjectName(u"groupBox_cut_motor")
        self.groupBox_cut_motor.setGeometry(QRect(10, 500, 331, 171))
        self.pushButton_reset_cut_motor = QPushButton(self.groupBox_cut_motor)
        self.pushButton_reset_cut_motor.setObjectName(u"pushButton_reset_cut_motor")
        self.pushButton_reset_cut_motor.setGeometry(QRect(10, 20, 41, 23))
        self.pushButton_anticlockwise_distance_cut_motor = QPushButton(self.groupBox_cut_motor)
        self.pushButton_anticlockwise_distance_cut_motor.setObjectName(u"pushButton_anticlockwise_distance_cut_motor")
        self.pushButton_anticlockwise_distance_cut_motor.setGeometry(QRect(110, 80, 71, 23))
        self.pushButton_clockwise_distance_cut_motor = QPushButton(self.groupBox_cut_motor)
        self.pushButton_clockwise_distance_cut_motor.setObjectName(u"pushButton_clockwise_distance_cut_motor")
        self.pushButton_clockwise_distance_cut_motor.setGeometry(QRect(110, 50, 71, 23))
        self.lineEdit_speed_cut_motor = QLineEdit(self.groupBox_cut_motor)
        self.lineEdit_speed_cut_motor.setObjectName(u"lineEdit_speed_cut_motor")
        self.lineEdit_speed_cut_motor.setGeometry(QRect(10, 50, 41, 21))
        self.lineEdit_speed_cut_motor.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lineEdit_distance_cut_motor = QLineEdit(self.groupBox_cut_motor)
        self.lineEdit_distance_cut_motor.setObjectName(u"lineEdit_distance_cut_motor")
        self.lineEdit_distance_cut_motor.setGeometry(QRect(10, 80, 41, 21))
        self.lineEdit_distance_cut_motor.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.label_4 = QLabel(self.groupBox_cut_motor)
        self.label_4.setObjectName(u"label_4")
        self.label_4.setGeometry(QRect(50, 50, 61, 21))
        self.label_4.setFont(font1)
        self.label_4.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.label_5 = QLabel(self.groupBox_cut_motor)
        self.label_5.setObjectName(u"label_5")
        self.label_5.setGeometry(QRect(50, 80, 61, 21))
        self.label_5.setFont(font1)
        self.label_5.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.pushButton_clockwise_sustain_cut_motor = QPushButton(self.groupBox_cut_motor)
        self.pushButton_clockwise_sustain_cut_motor.setObjectName(u"pushButton_clockwise_sustain_cut_motor")
        self.pushButton_clockwise_sustain_cut_motor.setGeometry(QRect(240, 50, 81, 23))
        self.pushButton_anticlockwise_sustain_cut_motor = QPushButton(self.groupBox_cut_motor)
        self.pushButton_anticlockwise_sustain_cut_motor.setObjectName(u"pushButton_anticlockwise_sustain_cut_motor")
        self.pushButton_anticlockwise_sustain_cut_motor.setGeometry(QRect(240, 80, 81, 23))
        self.pushButton_stop_cut_motor = QPushButton(self.groupBox_cut_motor)
        self.pushButton_stop_cut_motor.setObjectName(u"pushButton_stop_cut_motor")
        self.pushButton_stop_cut_motor.setGeometry(QRect(60, 20, 41, 23))
        self.pushButton_stop_cut_motor.setStyleSheet(u"color: rgb(255, 0, 0);\n"
"background-color: rgb(255, 255, 127);")
        self.pushButton_get_data_cut_motor = QPushButton(self.groupBox_cut_motor)
        self.pushButton_get_data_cut_motor.setObjectName(u"pushButton_get_data_cut_motor")
        self.pushButton_get_data_cut_motor.setGeometry(QRect(110, 20, 71, 23))
        self.pushButton_stop_data_cut_motor = QPushButton(self.groupBox_cut_motor)
        self.pushButton_stop_data_cut_motor.setObjectName(u"pushButton_stop_data_cut_motor")
        self.pushButton_stop_data_cut_motor.setGeometry(QRect(190, 20, 61, 23))
        self.pushButton_reset_data_cut_motor = QPushButton(self.groupBox_cut_motor)
        self.pushButton_reset_data_cut_motor.setObjectName(u"pushButton_reset_data_cut_motor")
        self.pushButton_reset_data_cut_motor.setGeometry(QRect(260, 20, 61, 23))
        self.pushButton_reset_data_cut_motor.setStyleSheet(u"color: rgb(255, 0, 0);\n"
"background-color: rgb(255, 255, 127);")
        self.lineEdit_file_name_cut_motor = QLineEdit(self.groupBox_cut_motor)
        self.lineEdit_file_name_cut_motor.setObjectName(u"lineEdit_file_name_cut_motor")
        self.lineEdit_file_name_cut_motor.setGeometry(QRect(100, 140, 61, 23))
        self.lineEdit_file_name_cut_motor.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.pushButton_select_folder_cut_motor = QPushButton(self.groupBox_cut_motor)
        self.pushButton_select_folder_cut_motor.setObjectName(u"pushButton_select_folder_cut_motor")
        self.pushButton_select_folder_cut_motor.setGeometry(QRect(10, 140, 81, 23))
        self.pushButton_save_data_cut_motor = QPushButton(self.groupBox_cut_motor)
        self.pushButton_save_data_cut_motor.setObjectName(u"pushButton_save_data_cut_motor")
        self.pushButton_save_data_cut_motor.setGeometry(QRect(240, 140, 81, 23))
        self.label_6 = QLabel(self.groupBox_cut_motor)
        self.label_6.setObjectName(u"label_6")
        self.label_6.setGeometry(QRect(170, 140, 61, 23))
        self.label_6.setFont(font1)
        self.label_6.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.label_folder = QLabel(self.groupBox_cut_motor)
        self.label_folder.setObjectName(u"label_folder")
        self.label_folder.setGeometry(QRect(10, 110, 311, 23))
        self.label_folder.setFont(font1)
        self.label_folder.setStyleSheet(u"")
        self.label_folder.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.label_message_cut_motor = QLabel(Form)
        self.label_message_cut_motor.setObjectName(u"label_message_cut_motor")
        self.label_message_cut_motor.setGeometry(QRect(20, 810, 301, 61))
        self.label_message_cut_motor.setFont(font)
        self.label_message_cut_motor.setStyleSheet(u"background-color: rgb(170, 255, 255);")
        self.label_message_cut_motor.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.label_message_penetration_motor = QLabel(Form)
        self.label_message_penetration_motor.setObjectName(u"label_message_penetration_motor")
        self.label_message_penetration_motor.setGeometry(QRect(20, 740, 301, 61))
        self.label_message_penetration_motor.setFont(font)
        self.label_message_penetration_motor.setStyleSheet(u"background-color: rgb(170, 255, 255);")
        self.label_message_penetration_motor.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.label_message_proximity_switch = QLabel(Form)
        self.label_message_proximity_switch.setObjectName(u"label_message_proximity_switch")
        self.label_message_proximity_switch.setGeometry(QRect(1220, 120, 171, 61))
        self.label_message_proximity_switch.setFont(font)
        self.label_message_proximity_switch.setStyleSheet(u"background-color: rgb(255, 255, 127);")
        self.label_message_proximity_switch.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.QChartview_penetration_motor = QChartView(Form)
        self.QChartview_penetration_motor.setObjectName(u"QChartview_penetration_motor")
        self.QChartview_penetration_motor.setGeometry(QRect(410, 20, 911, 31))
        self.QChartview_cut_motor = QChartView(Form)
        self.QChartview_cut_motor.setObjectName(u"QChartview_cut_motor")
        self.QChartview_cut_motor.setGeometry(QRect(410, 190, 911, 651))

        self.retranslateUi(Form)

        QMetaObject.connectSlotsByName(Form)
    # setupUi

    def retranslateUi(self, Form):
        Form.setWindowTitle(QCoreApplication.translate("Form", u"Form", None))
        self.groupBox.setTitle(QCoreApplication.translate("Form", u"A\u2014\u2014\u63a5\u8fd1\u5f00\u5173", None))
        self.pushButton_open_com_proximity_switch.setText(QCoreApplication.translate("Form", u"\u6253\u5f00\u4e32\u53e3", None))
        self.pushButton_close_com_proximity_switch.setText(QCoreApplication.translate("Form", u"\u5173\u95ed\u4e32\u53e3", None))
        self.groupBox_4.setTitle(QCoreApplication.translate("Form", u"B\u2014\u2014\u7535\u673a", None))
        self.pushButton_open_com_motor.setText(QCoreApplication.translate("Form", u"\u6253\u5f00\u4e32\u53e3", None))
        self.pushButton_close_com_motor.setText(QCoreApplication.translate("Form", u"\u5173\u95ed\u4e32\u53e3", None))
        self.groupBox_2_sensor.setTitle(QCoreApplication.translate("Form", u"C\u2014\u2014\u4e8c\u7ef4\u4f20\u611f\u5668", None))
        self.pushButton_open_com_2_sensor.setText(QCoreApplication.translate("Form", u"\u6253\u5f00\u4e32\u53e3", None))
        self.pushButton_close_com_2_sensor.setText(QCoreApplication.translate("Form", u"\u5173\u95ed\u4e32\u53e3", None))
        self.pushButton_open_com_all.setText(QCoreApplication.translate("Form", u"\u6253\u5f00\u6240\u6709\u4e32\u53e3", None))
        self.pushButton_8.setText(QCoreApplication.translate("Form", u"\u5173\u95ed\u6240\u6709\u4e32\u53e3", None))
        self.label_message.setText(QCoreApplication.translate("Form", u"TextLabel", None))
        self.groupBox_penetration_motor.setTitle(QCoreApplication.translate("Form", u"\u8d2f\u5165\u7535\u673a", None))
        self.pushButton_reset_penetration_motor.setText(QCoreApplication.translate("Form", u"\u5f52\u96f6", None))
        self.pushButton_down_distance_penetration_motor.setText(QCoreApplication.translate("Form", u"\u4e0b\u964d", None))
        self.pushButton_up_distance_penetration_motor.setText(QCoreApplication.translate("Form", u"\u4e0a\u5347", None))
        self.lineEdit_speed_penetration_motor.setText(QCoreApplication.translate("Form", u"7", None))
        self.lineEdit_distance_penetration_motor.setText(QCoreApplication.translate("Form", u"10", None))
        self.label_2.setText(QCoreApplication.translate("Form", u"mm/s", None))
        self.label_3.setText(QCoreApplication.translate("Form", u"mm", None))
        self.pushButton_up_sustain_penetration_motor.setText(QCoreApplication.translate("Form", u"\u4e0a\u5347\u6301\u7eed", None))
        self.pushButton_anticlockwise_sustain_penetration_motor.setText(QCoreApplication.translate("Form", u"\u4e0b\u964d\u6301\u7eed", None))
        self.pushButton_stop_data_penetration_motor.setText(QCoreApplication.translate("Form", u"\u505c\u6b62\u83b7\u53d6", None))
        self.pushButton_get_data_penetration_motor.setText(QCoreApplication.translate("Form", u"\u83b7\u53d6\u6570\u636e", None))
        self.pushButton_stop_penetration_motor.setText(QCoreApplication.translate("Form", u"\u505c\u6b62", None))
        self.pushButton_reset_data_penetration_motor.setText(QCoreApplication.translate("Form", u"\u91cd\u7f6e\u6570\u636e", None))
        self.pushButton_refresh_com.setText(QCoreApplication.translate("Form", u"\u5237\u65b0\u4e32\u53e3", None))
        self.groupBox_cut_motor.setTitle(QCoreApplication.translate("Form", u"\u526a\u5207\u7535\u673a", None))
        self.pushButton_reset_cut_motor.setText(QCoreApplication.translate("Form", u"\u590d\u4f4d", None))
        self.pushButton_anticlockwise_distance_cut_motor.setText(QCoreApplication.translate("Form", u"\u9006\u65f6\u9488", None))
        self.pushButton_clockwise_distance_cut_motor.setText(QCoreApplication.translate("Form", u"\u987a\u65f6\u9488", None))
        self.lineEdit_speed_cut_motor.setText(QCoreApplication.translate("Form", u"3", None))
        self.lineEdit_distance_cut_motor.setText(QCoreApplication.translate("Form", u"45", None))
        self.label_4.setText(QCoreApplication.translate("Form", u"degree/s", None))
        self.label_5.setText(QCoreApplication.translate("Form", u"degree", None))
        self.pushButton_clockwise_sustain_cut_motor.setText(QCoreApplication.translate("Form", u"\u987a\u65f6\u9488\u6301\u7eed", None))
        self.pushButton_anticlockwise_sustain_cut_motor.setText(QCoreApplication.translate("Form", u"\u9006\u65f6\u9488\u6301\u7eed", None))
        self.pushButton_stop_cut_motor.setText(QCoreApplication.translate("Form", u"\u505c\u6b62", None))
        self.pushButton_get_data_cut_motor.setText(QCoreApplication.translate("Form", u"\u83b7\u53d6\u6570\u636e", None))
        self.pushButton_stop_data_cut_motor.setText(QCoreApplication.translate("Form", u"\u505c\u6b62\u83b7\u53d6", None))
        self.pushButton_reset_data_cut_motor.setText(QCoreApplication.translate("Form", u"\u91cd\u7f6e\u6570\u636e", None))
        self.lineEdit_file_name_cut_motor.setText(QCoreApplication.translate("Form", u"01", None))
        self.pushButton_select_folder_cut_motor.setText(QCoreApplication.translate("Form", u"\u9009\u62e9\u6587\u4ef6\u5939", None))
        self.pushButton_save_data_cut_motor.setText(QCoreApplication.translate("Form", u"\u4fdd\u5b58\u5f53\u524d\u56fe\u8868", None))
        self.label_6.setText(QCoreApplication.translate("Form", u"\u6587\u4ef6\u540d\u79f0", None))
        self.label_folder.setText(QCoreApplication.translate("Form", u"\u6587\u4ef6\u5730\u5740", None))
        self.label_message_cut_motor.setText(QCoreApplication.translate("Form", u"\u526a\u5207\u7535\u673a\u4f4d\u7f6e", None))
        self.label_message_penetration_motor.setText(QCoreApplication.translate("Form", u"\u8d2f\u5165\u7535\u673a\u4f4d\u7f6e", None))
        self.label_message_proximity_switch.setText(QCoreApplication.translate("Form", u"\u63a5\u8fd1\u5f00\u5173\u4f4d\u7f6e", None))
    # retranslateUi


"""
电机控制包
包含通信、控制器和工具模块
"""

from .communication import CutMotorCommunication
from .controller import CutMotorController
from .utils import setup_logging

__version__ = '1.0.0'
__author__ = 'Your Name'

# 导出主要的类和函数，使得可以直接从包中导入
__all__ = [
    'MotorCommunication',
    'MotorController',
    'setup_logging',
] 
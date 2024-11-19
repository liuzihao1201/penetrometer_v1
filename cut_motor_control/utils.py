import logging
import os
from datetime import datetime

def setup_logging(base_path=None):
    """设置日志"""
    if base_path is None:
        base_path = os.getcwd()
        
    logs_dir = os.path.join(base_path, 'logs')
    if not os.path.exists(logs_dir):
        os.makedirs(logs_dir)
        
    log_file = os.path.join(logs_dir, f'motor_control_{datetime.now().strftime("%Y%m%d")}.log')
    
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(message)s',
        handlers=[
            logging.FileHandler(log_file, encoding='utf-8'),
            logging.StreamHandler()
        ]
    ) 
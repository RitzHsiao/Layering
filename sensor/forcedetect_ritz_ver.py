import serial
import time
import re
import numpy as np
import matplotlib.pyplot as plt
import os
import math
import single_pro_euler

from csv_writer import SensorCSVWriterThread # 自寫的儲存data多線程
from force_sensor_ritz import ForceSensorReader # 自寫的sensor讀取函數

from config import SERIAL_PORT, BAUD_RATE, INTERVAL
from config import FORCE_DATA_DIR, CONNECT_SENSOR_FILE, RETURN_ROBOT_FILE, RETURN_TRAJECTORY_FILE


# 初始化檔案
def initialize_files():
    os.makedirs(FORCE_DATA_DIR, exist_ok=True)
    for file in [CONNECT_SENSOR_FILE, RETURN_ROBOT_FILE, RETURN_TRAJECTORY_FILE]:
        with open(file, "w") as f:
            f.write("0")
    print("檔案初始化成功")

# 記錄數據
def log_force_data(timestamp, fx, fy, fz, force, trial_count):
    raw_data_file = os.path.join(FORCE_DATA_DIR, f"datadetect_{trial_count}.txt")
    force_file = os.path.join(FORCE_DATA_DIR, f"F_line_{trial_count}.txt")
    with open(raw_data_file, "a") as raw_file:
        raw_file.write(f"{timestamp:.2f}s\t{fx:.3f}\t{fy:.3f}\t{fz:.3f}\n")
    with open(force_file, "a") as f_file:
        f_file.write(f"{timestamp:.2f}s\t{force:.3f} N\n")


if __name__ == '__main__':

    initialize_files()  # 初始化負責與手臂C++溝通的檔案

    # 建立sensor讀取物件
    reader = ForceSensorReader(port=SERIAL_PORT, baudrate=BAUD_RATE, interval=INTERVAL)
    writer = SensorCSVWriterThread('force_data_ritz')

    if not reader.connect():
        raise RuntimeError("無法建立 Serial 連線，請確認感測器是否接好。")

    if not reader.wait_for_valid_response():
        raise RuntimeError("感測器未傳回有效資料，請將力感測器重新歸零。")

    start_time = time.time()
    timestamp = time.time() - start_time

    while (timestamp < 100):
        data = reader.read_once(start_time)
        print(data.F)

    reader.close()

'''
if detect_contact(force):
    update_robot_status(1)  # 回傳機械手臂：接觸到研磨點
    print("Contact detected!")
    time.sleep(1.5)  # 模擬回應時間
    trial_count += 1  # 進入下一次測試
    times, forces = [], []
    start_time = time.time()
'''
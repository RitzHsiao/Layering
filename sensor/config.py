'''
紀錄 force sensor 需要用到的參數
'''

# === 感測器參數 ===
SERIAL_PORT = "COM3"            # 連接到 Axia80 的 COM 埠（透過 USB） # 之前是 COM6
BAUD_RATE = 115200              # 固定波特率，根據 Axia80 RS422 協定
INTERVAL = 0.1                  # 每次讀取間隔（秒）


# === 資料儲存 ===
FORCE_DATA_DIR = "force_data_ritz"
CONNECT_SENSOR_FILE = "connect_sensor.txt"
RETURN_ROBOT_FILE = "return_robot.txt"
RETURN_TRAJECTORY_FILE = "return_trajectory.txt"
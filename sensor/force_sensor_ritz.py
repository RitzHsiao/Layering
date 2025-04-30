"""
force_sensor_ritz.py

定義 ForceSensorReader 類別，提供穩定、高可讀性的介面來與 ATI Axia80 力感測器溝通。
適用於使用 USB-to-Serial（RS422）接法的 Axia80，支援以 ASCII 格式傳回感測數據。

使用 PySerial，搭配非阻塞時間控制（使用 perf_counter 確保固定取樣間隔）。

開發者建議：
- 使用前確認 Serial Port 及 Baudrate 設定正確
- 預設感測器為 Streaming Mode，每行輸出一筆完整的 Fx~Tz 資料
- 建議搭配外部主控程式（如 main.py）進行輪詢與紀錄
"""

import serial
import time
import numpy as np

from dataclasses import dataclass


def format_sensor_data(data: "SensorData", with_unit: bool = True) -> str:
    """
    將 SensorData 轉換為可讀性高的字串格式。

    Args:
        data (SensorData): 要格式化的感測資料
        with_unit (bool): 是否顯示單位（N, Nm）

    Returns:
        str: 格式化後的資料字串
    """
    if with_unit:
        return (
            f"[{data.timestamp:>6.2f}s] "
            f"F: ({data.fx:>6.2f} N, {data.fy:>6.2f} N, {data.fz:>6.2f} N) | "
            f"T: ({data.tx:>7.4f} Nm, {data.ty:>7.4f} Nm, {data.tz:>7.4f} Nm)"
        )
    else:
        return (
            f"[{data.timestamp:>6.2f}s] "
            f"F: ({data.fx:>6.2f}, {data.fy:>6.2f}, {data.fz:>6.2f}) | "
            f"T: ({data.tx:>7.4f}, {data.ty:>7.4f}, {data.tz:>7.4f})"
        )


@dataclass
class SensorData:
    timestamp: float
    fx: float
    fy: float
    fz: float
    tx: float
    ty: float
    tz: float

    def __str__(self):
        return format_sensor_data(self)

    @property
    def F(self) -> float:
        """合力大小（N）"""
        return np.sqrt(self.fx ** 2 + self.fy ** 2 + self.fz ** 2)


class ForceSensorReader:
    def __init__(self, port="COM3", baudrate=115200, interval=0.1):
        self.port = port
        self.baudrate = baudrate
        self.interval = interval  # seconds
        self.ser = None

    def connect(self):
        """
            嘗試建立與力感測器的 Serial 連線。

            此方法會使用物件初始化時提供的 port 和 baudrate，
            並設定標準的串列通訊參數，包括：
                - 8 資料位元
                - 無奇偶校驗
                - 1 個停止位元
                - 禁用軟體與硬體流控制（xonxoff, rtscts, dsrdtr 全設為 False）
                - 設定讀寫 timeout 為 0.1 秒（非阻塞）

            適用於 ATI Axia80 感測器之 RS422 over USB 通訊情境

            Returns:
                bool: 若成功建立連線回傳 True，否則回傳 False（self.ser 會為 None）。
           """
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,

                # 資料格式設定
                bytesize=serial.EIGHTBITS,  # number of bits per bytes
                parity=serial.PARITY_NONE,  # set parity check
                stopbits=serial.STOPBITS_ONE,  # number of stop bits

                # 通訊控制
                timeout=0.1,  # non-block read 0.5s
                write_timeout=0.1,  # timeout for write 0.5s
                xonxoff=False,  # disable software flow control
                rtscts=False,  # disable hardware (RTS/CTS) flow control
                dsrdtr=False  # disable hardware (DSR/DTR) flow control
            )
            print(f"成功連接到：{self.port}")
            return True
        except Exception as e:
            print(f"Serial 連接失敗: {e}")
            self.ser = None
            return False

    def wait_for_valid_response(self, timeout=5.0):
        """
        持續等待感測器回應一筆有效封包，最多等待 timeout 秒。

        Args:
            timeout (float): 最多等待時間（秒）

        Returns:
            bool: 成功取得有效封包回傳 True，否則 False。
        """

        print("等待感測器傳回有效資料...")
        start = time.perf_counter()
        while time.perf_counter() - start < timeout:
            data = self.read_once(-1) # -1 為僅共測試用 不須紀錄timestamp
            if data:
                print("成功取得第一筆有效資料")
                return True
            time.sleep(0.01)
        print("[超時]：未取得任何有效封包")
        return False

    @staticmethod
    def parse_packet(start_time, response: bytes):
        """
                將感測器回傳的原始位元資料解析為 SensorData 結構
                (timestamp, Fx, Fy, Fz, Tx, Ty, Tz)

               Args:
                    start_time (float): 程式啟動的起始時間（透過 time.time() 或 time.perf_counter()）。
                        - 若傳入正數（>0），則 timestamp = 現在時間 - start_time
                        - 若為 0 或負數，則 timestamp 預設為 -1，表示測試階段不記錄時間
                    response (bytes): 感測器單筆回傳的資料，格式為 ASCII 編碼字串

               Returns:
                    SensorData or None: 若封包格式與單位正確，則回傳解析後的 SensorData；
                    若格式錯誤或解析失敗，回傳 None。
        """
        try:
            decoded = response.decode('utf-8').strip()
            if decoded.startswith('>'):
                parts = decoded[1:].split()  # 移除開頭 '>' 再切割
            else:
                parts = decoded.split()

            if len(parts) != 12:
                print("收取到的資訊不足")
                return None
            if parts[1] != 'N' or parts[3] != 'N' or parts[5] != 'N':
                print("單位不符，資料可能有誤")
                return None
            if parts[7] != 'Nm' or parts[9] != 'Nm' or parts[11] != 'Nm':
                print("單位不符，資料可能有誤")
                return None
            fx, fy, fz = map(float, [parts[0], parts[2], parts[4]])
            tx, ty, tz = map(float, [parts[6], parts[8], parts[10]])
            if start_time>0:
                timestamp = time.time() - start_time
            else:
                timestamp = -1.0 # -1 為僅共測試用 不須紀錄timestamp

            return SensorData(
                timestamp=timestamp,
                fx=fx, fy=fy, fz=fz,
                tx=tx, ty=ty, tz=tz
            )
        except Exception as e:
            print(f"解封包錯誤: {e}")
            return None

    def read_once(self, start_time, retry=3):
        """
            嘗試讀取一筆完整資料，最多重試 retry 次。

            Args:
                start_time (float): 程式啟動時間，用於計算相對 timestamp。
                    - 若為正數，timestamp = 現在時間 - start_time。
                    - 若為 0 或負數，timestamp 將預設為 -1。
                retry (int, optional): 最大重試次數。預設為 3。

            Returns:
                SensorData or None: 若成功解析則回傳 SensorData，否則回傳 None。
        """
        if not self.ser or not self.ser.is_open:
            print("Serial未建立(未呼叫connect())或已關閉")
            return None

        for attempt in range(retry):
            response = self.ser.readline()
            data = ForceSensorReader.parse_packet(start_time, response)
            if data:
                print(f"取得資料:{data}")
                return data
            else:
                print(f"第 {attempt + 1} 次嘗試失敗，未取得完整封包")
                time.sleep(0.01)  # 等一下再試

        print("retry後仍未讀到有效資料")
        return None

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("Serial 已關閉")




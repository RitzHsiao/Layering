import csv
import threading
import queue
from pathlib import Path
from datetime import datetime
from dataclasses import asdict


class SensorCSVWriterThread:
    """
    後台寫入 SensorData 的 writer thread。使用 queue 非同步寫入，避免阻塞主感測器讀取。
    """

    def __init__(self, folder):
        self.folder = Path(folder)
        self.folder.mkdir(parents=True, exist_ok=True)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.file_path = self.folder / f"trial_{timestamp}.csv"

        self.data_queue = queue.Queue()
        self.running = False  # 控制 loop 是否繼續
        self.thread = None  # 執行 _write_loop 的背景寫入者

        # 建立檔案並寫入表頭
        with open(self.file_path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=["timestamp", "fx", "fy", "fz", "tx", "ty", "tz", "F"])
            writer.writeheader()

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._write_loop, daemon=True)  # daemon=True 當主程式（main thread）結束時，daemon thread 會被強制中斷
        self.thread.start()
        print(f"Writer thread started, logging to: {self.file_path}")

    def stop(self):
        self.running = False  # 如果queue還有資料，還是會寫完 >>「你寫到沒有資料為止就可以下班嚕」
        self.thread.join()
        print("Writer thread stopped")

    def log(self, data):
        self.data_queue.put(data)

    def _write_loop(self):
        with open(self.file_path, "a", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=["timestamp", "fx", "fy", "fz", "tx", "ty", "tz", "F"])
            while self.running or not self.data_queue.empty():
                try:
                    data = self.data_queue.get(timeout=0.1)
                    row = asdict(data)
                    row["F"] = data.F
                    writer.writerow(row)
                    f.flush()
                except queue.Empty:
                    continue  # 等資料丟進來

    def get_path(self):
        return str(self.file_path)

import serial
import threading
import time
import struct
import os

# === 发送协议常量 (30字节) ===
TX_HEAD_1 = 0x55
TX_HEAD_2 = 0xAA
TX_LEN    = 0x1E      # 30字节
TX_CMD    = 0x01      # 发送命令

class CANDriver:
    def __init__(self, port="/dev/ttyACM0", baudrate=921600):
        # 自动提权
        os.system(f"sudo chmod 777 {port}")
        
        try:
            self.ser = serial.Serial(port, baudrate, timeout=0.02)
            print(f"[CAN Driver] 串口 {port} 连接成功")
        except Exception as e:
            print(f"[CAN Driver] 打开失败: {e}")
            self.ser = None

        self.running = True
        self.rx_callback = None
        
        if self.ser and self.ser.isOpen():
            self.thread = threading.Thread(target=self._rx_thread)
            self.thread.daemon = True
            self.thread.start()

    def set_rx_callback(self, callback):
        """回调函数形式: callback(can_id, data_bytes)"""
        self.rx_callback = callback

    def send_can_frame(self, can_id, data_bytes):
        """
        发送 CAN 帧 (30字节格式)
        完全符合 CSV 文档: 55 AA 1E 01 ... Data在21偏移
        """
        if not self.ser: return
        if len(data_bytes) != 8: return

        # 初始化全0缓冲区
        buffer = bytearray(30)
        
        # 1. 帧头与配置
        buffer[0] = TX_HEAD_1
        buffer[1] = TX_HEAD_2
        buffer[2] = TX_LEN
        buffer[3] = TX_CMD
        
        # 发送次数(1) 和 间隔(0)
        buffer[4] = 0x01 
        
        # ID类型 (0=标准帧)
        buffer[12] = 0x00
        
        # 2. CAN ID (Little Endian, 4 Bytes) -> CSV Data13-16
        buffer[13:17] = int(can_id).to_bytes(4, byteorder='little')
        
        # 帧类型(0=数据) 和 DLC(8)
        buffer[17] = 0x00
        buffer[18] = 0x08
        
        # 3. 数据段 (8 Bytes) -> CSV Data21-28
        for i in range(8):
            buffer[21+i] = data_bytes[i]
            
        # 4. 校验/尾部 (根据文档通常是校验或固定值，旧代码习惯用 0x88)
        buffer[29] = 0x88 

        try:
            self.ser.write(buffer)
        except Exception as e:
            print(f"Tx Error: {e}")

    def _rx_thread(self):
        """
        接收线程: 解析 16 字节帧 (AA ... 55)
        参考 CSV: eg: AA 11 08 ... 55
        """
        buffer = bytearray()
        while self.running and self.ser:
            try:
                waiting = self.ser.inWaiting()
                if waiting > 0:
                    buffer.extend(self.ser.read(waiting))
                    
                    # 循环解析缓冲区
                    while len(buffer) >= 16:
                        # 严格校验头尾: Head=AA, Tail=55
                        if buffer[0] == 0xAA and buffer[15] == 0x55:
                            # === 帧解析 ===
                            frame = buffer[:16]
                            buffer = buffer[16:] # 移出已处理数据
                            
                            # 提取 ID (Byte 3-6, Little Endian)
                            can_id = int.from_bytes(frame[3:7], byteorder='little')
                            
                            # 提取 Data (Byte 7-14, 8 Bytes)
                            payload = frame[7:15]
                            
                            # 回调
                            if self.rx_callback:
                                self.rx_callback(can_id, payload)
                        else:
                            # 帧头不对，滑动窗口丢弃 1 字节
                            buffer.pop(0)
                else:
                    time.sleep(0.002)
            except Exception as e:
                print(f"Rx Error: {e}")
                time.sleep(0.1)
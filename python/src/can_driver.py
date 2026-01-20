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
        """发送 CAN 帧 (30字节格式)"""
        if not self.ser or not self.running: 
            return
        if len(data_bytes) != 8: 
            return

        # 初始化全0缓冲区
        buffer = bytearray(30)
        buffer[0] = TX_HEAD_1
        buffer[1] = TX_HEAD_2
        buffer[2] = TX_LEN
        buffer[3] = TX_CMD
        buffer[4] = 0x01 
        buffer[12] = 0x00
        buffer[13:17] = int(can_id).to_bytes(4, byteorder='little')
        buffer[17] = 0x00
        buffer[18] = 0x08
        for i in range(8):
            buffer[21+i] = data_bytes[i]
        buffer[29] = 0x88 

        try:
            self.ser.write(buffer)
        except Exception as e:
            print(f"Tx Error: {e}")

    def _rx_thread(self):
        """接收线程: 解析 16 字节帧 (AA ... 55)"""
        buffer = bytearray()
        while self.running and self.ser:
            try:
                waiting = self.ser.inWaiting()
                if waiting > 0:
                    buffer.extend(self.ser.read(waiting))
                    
                    while len(buffer) >= 16:
                        if buffer[0] == 0xAA and buffer[15] == 0x55:
                            frame = buffer[:16]
                            buffer = buffer[16:]
                            
                            can_id = int.from_bytes(frame[3:7], byteorder='little')
                            payload = frame[7:15]
                            
                            if self.rx_callback:
                                self.rx_callback(can_id, payload)
                        else:
                            buffer.pop(0)
                else:
                    time.sleep(0.002)
            except Exception as e:
                if self.running:  # 只在运行时打印错误
                    print(f"Rx Error: {e}")
                time.sleep(0.1)
        
        # 线程退出时关闭串口
        if self.ser and self.ser.isOpen():
            try:
                self.ser.close()
                print("[CAN Driver] 串口已关闭")
            except:
                pass
    
    def __del__(self):
        """析构函数：确保资源释放"""
        self.running = False
        if hasattr(self, 'ser') and self.ser and self.ser.isOpen():
            try:
                self.ser.close()
            except:
                pass
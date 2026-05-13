#!/usr/bin/env python3
import serial

import time
import sys

# --- 配置参数 ---
PORT = '/dev/ttyUSB0'
BAUDRATE = 115200
SHOW_RAW_DATA = False  # 设置为 True 可以打印传感器发来的每一行原始字符串

def main():
    print(f"🔄 正在尝试连接串口 {PORT} (波特率: {BAUDRATE})...")
    
    try:
        # 打开串口
        ser = serial.Serial(PORT, BAUDRATE, timeout=1)
        print(f"✅ 串口打开成功！等待接收数据...\n")
    except serial.SerialException as e:
        print(f"❌ 串口打开失败: {e}")
        print("请检查设备是否插入，或尝试运行: sudo chmod 777 /dev/ttyUSB0")
        sys.exit(1)

    # 用于缓存一帧完整的数据
    data_frame = {
        'yaw': None,
        'weiyi_x': None,
        'weiyi_y': None,
        'az': None
    }

    try:
        while True:
            # 检查缓冲区是否有数据
            if ser.in_waiting > 0:
                # 读取一行并解码，忽略无法解码的乱码，去除首尾的回车换行符
                raw_line = ser.readline().decode('utf-8', errors='ignore').strip()
                
                if not raw_line:
                    continue
                    
                if SHOW_RAW_DATA:
                    print(f"[RAW] {raw_line}")

                # 按照 ':' 分割键值对
                parts = raw_line.split(':')
                if len(parts) >= 2:
                    # 提取键和值，并将键转换为小写（对应C++中的 ::tolower）
                    key = parts[0].strip().lower()
                    value_str = parts[1].strip()

                    try:
                        value = float(value_str)
                    except ValueError:
                        print(f"⚠️ 数据转换失败: '{value_str}' 不是有效的数字")
                        continue

                    # 如果解析到的键是我们关心的，存入字典
                    if key in data_frame:
                        data_frame[key] = value

                    # 检查是否凑齐了完整的一帧数据 (对应 C++ 中的 data_count_ == 0x0F)
                    if all(v is not None for v in data_frame.values()):
                        # 格式化打印完整数据
                        print(f"🎯 [完整数据] 位置:(X: {data_frame['weiyi_x']:6.2f}, "
                              f"Y: {data_frame['weiyi_y']:6.2f}) | "
                              f"偏航角 Yaw: {data_frame['yaw']:6.2f}° | "
                              f"Z轴加速度 Az: {data_frame['az']:6.2f}")
                        
                        # 清空字典，准备接收下一轮数据
                        for k in data_frame.keys():
                            data_frame[k] = None

            else:
                # 稍作延时，避免 CPU 占用过高
                time.sleep(0.005)

    except KeyboardInterrupt:
        print("\n🛑 收到中断信号，程序退出。")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("🔒 串口已关闭。")

if __name__ == '__main__':
    main()
import serial
import time
import struct
import sys

# === 配置 ===
PORT = "/dev/ttyACM0"
BAUD = 921600

def parse_gm6020(can_id, payload):
    """解析 GM6020/DJI 电机数据"""
    if len(payload) < 8: return
    
    # DJI 反馈格式 (大端): Angle(2) RPM(2) Current(2) Temp(1) Null(1)
    angle, rpm, current, temp = struct.unpack('>hhhB', payload[:7])
    
    # 计算物理角度 (0-8191 -> 0-360度)
    angle_deg = (angle / 8191.0) * 360.0
    
    print(f"   [ID: {hex(can_id)}] "
          f"RPM: {rpm:5d} | "
          f"Current: {current:5d} | "
          f"Temp: {temp:2d}°C | "
          f"Angle: {angle:5d} ({angle_deg:.1f}°)")

def main():
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.02)
        print(f"✅ 串口 {PORT} 打开成功")
        print("⚡ 正在监听 CAN 总线 (按 Ctrl+C 退出)...")
        print("-" * 60)
    except Exception as e:
        print(f"❌ 无法打开串口: {e}")
        return

    buffer = bytearray()

    while True:
        try:
            # 读取数据
            if ser.inWaiting() > 0:
                data = ser.read(ser.inWaiting())
                buffer.extend(data)

                # === 核心解析循环 ===
                while len(buffer) >= 16: # 帧长固定为 16
                    
                    # 1. 寻找帧头 55 AA
                    if buffer[0] == 0x55 and buffer[1] == 0xAA:
                        # 提取完整一帧 (16字节)
                        frame = buffer[:16]
                        buffer = buffer[16:] # 移出缓冲区
                        
                        # 2. 打印原始 Hex (调试用)
                        raw_hex = ' '.join([f"{b:02X}" for b in frame])
                        print(f"RAW: {raw_hex}")
                        
                        # 3. 提取 ID (Byte 4-7, 小端)
                        can_id = int.from_bytes(frame[4:8], byteorder='little')
                        
                        # 4. 提取 Payload (Byte 8-15)
                        payload = frame[8:16]
                        
                        # 5. 解析并显示
                        parse_gm6020(can_id, payload)
                        print("-" * 40)
                        
                    else:
                        # 如果不是 55 AA，丢弃第一个字节，继续寻找
                        buffer.pop(0)
            
            time.sleep(0.001)

        except KeyboardInterrupt:
            print("\n🛑 停止监听")
            break
        except Exception as e:
            print(f"\n❌ 错误: {e}")
            break
    
    if ser.isOpen():
        ser.close()

if __name__ == "__main__":
    main()
import serial
import time
import struct
import os

# === 配置 ===
PORT = "/dev/ttyACM0"
BAUD = 921600

# 电机状态跟踪
motor_states = {}

def parse_gm6020(can_id, payload):
    """解析 GM6020/DJI 电机数据并更新状态"""
    if len(payload) < 8:
        return
    
    # DJI 反馈格式 (大端): Angle(2) RPM(2) Current(2) Temp(1) Null(1)
    angle, rpm, current, temp = struct.unpack('>hhhB', payload[:7])
    
    # 计算物理角度 (0-8191 -> 0-360度)
    angle_deg = (angle / 8191.0) * 360.0
    
    # 推断电机ID (0x205->ID1, 0x206->ID2, ...)
    motor_id = can_id - 0x204 if can_id >= 0x205 else None
    
    if motor_id is None:
        return
    
    # 更新状态
    motor_states[motor_id] = {
        'angle': angle_deg,
        'rpm': rpm,
        'current': current,
        'temp': temp,
        'can_id': can_id
    }

def print_motor_states():
    """按 main.py 格式打印所有电机状态"""
    if not motor_states:
        return
    
    for motor_id in sorted(motor_states.keys()):
        state = motor_states[motor_id]
        
        # 格式: [IDx] Ang: current°→target° | Spd: current→target | Out: xxx | EN
        # 调试模式下没有目标值，用当前值代替
        print(f"[ID{motor_id}] "
              f"Ang: {state['angle']:6.1f}°→{state['angle']:6.1f}° | "
              f"Spd: {state['rpm']:5d}→{state['rpm']:6.1f} | "
              f"Out: {state['current']:6d} | "
              f"EN")
    print()  # 空行分隔

def main():
    # 自动提权
    os.system(f"sudo chmod 777 {PORT}")
    
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.02)
        print(f"✅ 串口 {PORT} 打开成功")
        print("⚡ 正在监听 CAN 总线 (按 Ctrl+C 退出)...")
        print("-" * 60)
    except Exception as e:
        print(f"❌ 无法打开串口: {e}")
        return

    buffer = bytearray()
    frame_count = 0

    while True:
        try:
            # 读取数据
            if ser.inWaiting() > 0:
                data = ser.read(ser.inWaiting())
                buffer.extend(data)

                # === 核心解析循环 ===
                while len(buffer) >= 16:  # 帧长固定为 16
                    
                    # 1. 寻找帧头 AA，帧尾 55
                    if buffer[0] == 0xAA and len(buffer) >= 16 and buffer[15] == 0x55:
                        # 提取完整一帧 (16字节)
                        frame = buffer[:16]
                        buffer = buffer[16:]  # 移出缓冲区
                        
                        # 2. 提取 ID (Byte 3-6, 小端)
                        can_id = int.from_bytes(frame[3:7], byteorder='little')
                        
                        # 3. 提取 Payload (Byte 7-14, 8字节数据段)
                        payload = frame[7:15]
                        
                        # 4. 解析并更新状态
                        parse_gm6020(can_id, payload)
                        
                        frame_count += 1
                        
                        # 每收到一组完整数据就打印一次
                        # (假设有2个电机，每2帧打印一次)
                        if frame_count % len(motor_states) == 0 and motor_states:
                            print_motor_states()
                        
                    else:
                        # 如果不是 AA 开头，丢弃第一个字节，继续寻找
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
#!/usr/bin/env python3
"""
GO-M8010-6 电机控制测试 - Python原生协议实现
使用定点数格式复现官方SDK功能
"""
import serial
import time
import struct
import math

gear_ratio = 6.33 
# CRC-CCITT 256项查表
CRC_TABLE = [
    0x0000, 0x1189, 0x2312, 0x329B, 0x4624, 0x57AD, 0x6536, 0x74BF,
    0x8C48, 0x9DC1, 0xAF5A, 0xBED3, 0xCA6C, 0xDBE5, 0xE97E, 0xF8F7,
    0x1081, 0x0108, 0x3393, 0x221A, 0x56A5, 0x472C, 0x75B7, 0x643E,
    0x9CC9, 0x8D40, 0xBFDB, 0xAE52, 0xDAED, 0xCB64, 0xF9FF, 0xE876,
    0x2102, 0x308B, 0x0210, 0x1399, 0x6726, 0x76AF, 0x4434, 0x55BD,
    0xAD4A, 0xBCC3, 0x8E58, 0x9FD1, 0xEB6E, 0xFAE7, 0xC87C, 0xD9F5,
    0x3183, 0x200A, 0x1291, 0x0318, 0x77A7, 0x662E, 0x54B5, 0x453C,
    0xBDCB, 0xAC42, 0x9ED9, 0x8F50, 0xFBEF, 0xEA66, 0xD8FD, 0xC974,
    0x4204, 0x538D, 0x6116, 0x709F, 0x0420, 0x15A9, 0x2732, 0x36BB,
    0xCE4C, 0xDFC5, 0xED5E, 0xFCD7, 0x8868, 0x99E1, 0xAB7A, 0xBAF3,
    0x5285, 0x430C, 0x7197, 0x601E, 0x14A1, 0x0528, 0x37B3, 0x263A,
    0xDECD, 0xCF44, 0xFDDF, 0xEC56, 0x98E9, 0x8960, 0xBBFB, 0xAA72,
    0x6306, 0x728F, 0x4014, 0x519D, 0x2522, 0x34AB, 0x0630, 0x17B9,
    0xEF4E, 0xFEC7, 0xCC5C, 0xDDD5, 0xA96A, 0xB8E3, 0x8A78, 0x9BF1,
    0x7387, 0x620E, 0x5095, 0x411C, 0x35A3, 0x242A, 0x16B1, 0x0738,
    0xFFCF, 0xEE46, 0xDCDD, 0xCD54, 0xB9EB, 0xA862, 0x9AF9, 0x8B70,
    0x8408, 0x9581, 0xA71A, 0xB693, 0xC22C, 0xD3A5, 0xE13E, 0xF0B7,
    0x0840, 0x19C9, 0x2B52, 0x3ADB, 0x4E64, 0x5FED, 0x6D76, 0x7CFF,
    0x9489, 0x8500, 0xB79B, 0xA612, 0xD2AD, 0xC324, 0xF1BF, 0xE036,
    0x18C1, 0x0948, 0x3BD3, 0x2A5A, 0x5EE5, 0x4F6C, 0x7DF7, 0x6C7E,
    0xA50A, 0xB483, 0x8618, 0x9791, 0xE32E, 0xF2A7, 0xC03C, 0xD1B5,
    0x2942, 0x38CB, 0x0A50, 0x1BD9, 0x6F66, 0x7EEF, 0x4C74, 0x5DFD,
    0xB58B, 0xA402, 0x9699, 0x8710, 0xF3AF, 0xE226, 0xD0BD, 0xC134,
    0x39C3, 0x284A, 0x1AD1, 0x0B58, 0x7FE7, 0x6E6E, 0x5CF5, 0x4D7C,
    0xC60C, 0xD785, 0xE51E, 0xF497, 0x8028, 0x91A1, 0xA33A, 0xB2B3,
    0x4A44, 0x5BCD, 0x6956, 0x78DF, 0x0C60, 0x1DE9, 0x2F72, 0x3EFB,
    0xD68D, 0xC704, 0xF59F, 0xE416, 0x90A9, 0x8120, 0xB3BB, 0xA232,
    0x5AC5, 0x4B4C, 0x79D7, 0x685E, 0x1CE1, 0x0D68, 0x3FF3, 0x2E7A,
    0xE70E, 0xF687, 0xC41C, 0xD595, 0xA12A, 0xB0A3, 0x8238, 0x93B1,
    0x6B46, 0x7ACF, 0x4854, 0x59DD, 0x2D62, 0x3CEB, 0x0E70, 0x1FF9,
    0xF78F, 0xE606, 0xD49D, 0xC514, 0xB1AB, 0xA022, 0x92B9, 0x8330,
    0x7BC7, 0x6A4E, 0x58D5, 0x495C, 0x3DE3, 0x2C6A, 0x1EF1, 0x0F78
]


def calc_crc_ccitt(data):
    """CRC-CCITT 16-bit LSB-first 计算 (官方算法)"""
    crc = 0
    for byte in data:
        crc = (crc >> 8) ^ CRC_TABLE[(crc ^ byte) & 0xff]
    return crc & 0xffff


def create_motor_command(motor_id=0, mode=1, tau=0.0, spd_des=0.0, pos_des=0.0, k_pos=0.0, k_spd=0.0):
    """
    创建GO-M8010-6电机命令包 (15字节数据 + 2字节CRC = 17字节)
    
    命令结构:
    - head[2]: 0xFE, 0xEE
    - mode: id(4bit) | status(3bit) | none(1bit)
    - RIS_Comd_t (12字节):
      - tor_des: int16_t (q8 format)
      - spd_des: int16_t (q7 format)
      - pos_des: int32_t (q15 format)
      - k_pos: uint16_t (q15 format)
      - k_spd: uint16_t (q15 format)
    - CRC16: uint16_t (2字节)
    """
    cmd = bytearray(17)
    
    # Header
    cmd[0] = 0xFE
    cmd[1] = 0xEE
    
    # Mode byte: id(4bit) | status(3bit) | none(1bit)
    cmd[2] = ((motor_id & 0x0F) | ((mode & 0x07) << 4))
    
    # RIS_Comd_t 结构 (12字节)
    # 将float转换为定点数
    tor_des_q8 = int(max(-32768, min(32767, tau * 256)))  # q8 format (2字节)，范围[-128, 127.99]
    spd_des_q7 = int(max(-32768, min(32767, spd_des * 256 /2 /math.pi )))  # q7 format (2字节)，范围[-256, 255.99]
    pos_des_q15 = int(max(-2147483648, min(2147483647, pos_des * 32768 / 2 / math.pi )))  # q15 format (4字节)，范围[-65536, 65535.99]
    k_pos_q15 = int(max(0, min(65535, k_pos * 1280))) & 0xFFFF  # q15 format (2字节)，范围[0, 1.0]
    k_spd_q15 = int(max(0, min(65535, k_spd * 1280))) & 0xFFFF  # q15 format (2字节)，范围[0, 1.0]
    
    # 打包成小端序
    cmd[3:5] = struct.pack('<h', tor_des_q8)      # int16_t tor_des
    cmd[5:7] = struct.pack('<h', spd_des_q7)      # int16_t spd_des
    cmd[7:11] = struct.pack('<i', pos_des_q15)    # int32_t pos_des
    cmd[11:13] = struct.pack('<H', k_pos_q15)     # uint16_t k_pos
    cmd[13:15] = struct.pack('<H', k_spd_q15)     # uint16_t k_spd
    
    # CRC (前15字节的CRC)
    crc = calc_crc_ccitt(cmd[:15])
    cmd[15] = crc & 0xFF
    cmd[16] = (crc >> 8) & 0xFF
    
    return bytes(cmd)


def parse_motor_feedback(data):
    """
    解析电机反馈数据（16字节）
    
    反馈结构:
    - head[2]: 0xFE, 0xEE
    - mode: id(4bit) | status(3bit) | none(1bit)
    - RIS_Fbk_t (11字节):
      - torque: int16_t (q8 format)
      - speed: int16_t (q7 format)
      - pos: int32_t (q15 format)
      - temp: int8_t
      - MError (3bit) | force (12bit) | none(1bit)
    - CRC16: uint16_t (2字节)
    """
    if len(data) < 16:
        return None
    
    try:
        motor_id = data[2] & 0x0F
        mode = (data[2] >> 4) & 0x07
        
        # 解析定点数格式
        torque_q8 = struct.unpack('<h', data[3:5])[0]  # int16_t torque (q8)
        speed_q7 = struct.unpack('<h', data[5:7])[0]   # int16_t speed (q7)
        pos_q15 = struct.unpack('<i', data[7:11])[0]   # int32_t pos (q15)
        temp = struct.unpack('<b', data[11:12])[0]     # int8_t temp
        
        # MError (3bit) + force (12bit) + reserved(1bit)
        byte12 = data[12]
        merror = byte12 & 0x07
        
        # 转换回float
        torque = torque_q8 / 256.0
        speed = speed_q7 *2 * math.pi /gear_ratio / 256.0
        pos = pos_q15  *360/gear_ratio / 32768.0
        
        return {
            'id': motor_id,
            'mode': mode,
            'torque': torque,
            'speed': speed,
            'pos': pos,
            'temp': temp,
            'merror': merror
        }
    except:
        return None


def main():
    port = '/dev/ttyUSB0'
    baudrate = 4000000
    
    try:
        ser = serial.Serial(port, baudrate, timeout=0.2)
        print(f"[✓] 打开串口: {port} @ {baudrate} baud\n")
    except Exception as e:
        print(f"[✗] 打开串口失败: {e}")
        return
    
    # 电机参数（与官方SDK相同）
    
    motor_id =[0, 1, 3] # 电机ID列表
    mode = 1 # FOC模式
    k_pos = 0.00 # 刚度系数
    k_spd = 0.00   # 阻尼系数
    
    target_speed = 0.0* gear_ratio  # -39.75 rad/s
    
    print(f"[Python] GO-M8010-6 电机控制测试（定点数协议）")
    print(f"[参数] ID={motor_id}, 模式=FOC, k_pos={k_pos}, k_spd={k_spd}")
    print(f"[目标] 速度={target_speed:.2f} rad/s\n")
    print("[执行] 发送15条电机转动命令...\n")
    
    count = 0
    success = 0
    
    try:
        while True:
            # 创建命令 (使用定点数格式)
          for i in range(len(motor_id)):  
            cmd = create_motor_command(
                motor_id=motor_id[i-1],
                mode=mode,
                tau=0.0,
                spd_des=target_speed,
                pos_des=180.0 * gear_ratio *math.pi /180,  # 位置保持不变
                k_pos=k_pos,
                k_spd=k_spd
            )
            
            # 发送
            ser.write(cmd)
            crc = calc_crc_ccitt(cmd[:15])
            count += 1
            
            # 接收反馈 (16字节)
            time.sleep(0.01)
            response = b''
            while len(response) < 16:
                chunk = ser.read(16 - len(response))
                if not chunk:
                    break
                response += chunk
            
            if len(response) >= 16:
                feedback = parse_motor_feedback(response)
                if feedback:
                    success += 1
                    print(f"[{count:2d}] id={feedback['id']} CRC=0x{crc:04X} ✓ | "
                          f"pos={feedback['pos']:9.2f} | "
                          f"spd={feedback['speed']:8.2f} | "
                          f"T={feedback['temp']:3d}°C | "
                          f"err={feedback['merror']}")#位置为角度，速度为弧度
                else:
                    print(f"[{count:2d}] CRC=0x{crc:04X} ✗ | [解析失败]")
            else:
                print(f"[{count:2d}] CRC=0x{crc:04X} ✗ | [无反馈]")
            
            time.sleep(0.05)
    
    except KeyboardInterrupt:
        print("\n[中断] 用户停止")
    
    finally:
        ser.close()
        print(f"\n[统计] 发送{count}条命令, 收到{success}次反馈 ({100*success/max(1,count):.1f}%)")
        if success == count:
            print("[✓] Python原生协议版本测试完成！电机成功响应！")
        else:
            print(f"[⚠] 部分命令未收到反馈，成功率 {100*success/max(1,count):.1f}%")


if __name__ == '__main__':
    main()

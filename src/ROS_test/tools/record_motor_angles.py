#!/usr/bin/env python3
"""
宇树电机角度采样记录工具
实时显示并记录4个电机的角度
"""

import rclpy
from rclpy.node import Node
from motor_control_ros2.msg import UnitreeGO8010State
import csv
import time
import math
from datetime import datetime

class MotorAngleRecorder(Node):
    def __init__(self):
        super().__init__('motor_angle_recorder')
        
        # 记录数据
        self.data_log = []
        self.motor_angles = {
            'strike_motor_L1': None,
            'strike_motor_L2': None,
            'strike_motor_R1': None,
            'strike_motor_R2': None
        }
        self.start_time = time.time()
        
        # 订阅电机状态
        self.subscription = self.create_subscription(
            UnitreeGO8010State,
            '/unitree_go8010_states',
            self.state_callback,
            10)
        
        # 计数器
        self.sample_count = 0
        
        print("\n" + "="*70)
        print("  宇树电机角度实时采样与记录")
        print("="*70)
        print("\n开始记录电机角度...")
        print("按 Ctrl+C 停止并保存数据\n")
        print(f"{'时间(s)':<10} {'L1(°)':<10} {'L2(°)':<10} {'R1(°)':<10} {'R2(°)':<10}")
        print("-" * 70)
        
    def state_callback(self, msg):
        """接收电机状态并记录"""
        if msg.joint_name in self.motor_angles:
            # 转换为角度
            angle_deg = math.degrees(msg.position)
            self.motor_angles[msg.joint_name] = angle_deg
            
            # 当收集到所有4个电机的数据时，记录一次
            if all(angle is not None for angle in self.motor_angles.values()):
                timestamp = time.time() - self.start_time
                
                # 记录数据
                data_row = {
                    'timestamp': timestamp,
                    'L1_deg': self.motor_angles['strike_motor_L1'],
                    'L2_deg': self.motor_angles['strike_motor_L2'],
                    'R1_deg': self.motor_angles['strike_motor_R1'],
                    'R2_deg': self.motor_angles['strike_motor_R2'],
                    'L1_rad': math.radians(self.motor_angles['strike_motor_L1']),
                    'L2_rad': math.radians(self.motor_angles['strike_motor_L2']),
                    'R1_rad': math.radians(self.motor_angles['strike_motor_R1']),
                    'R2_rad': math.radians(self.motor_angles['strike_motor_R2'])
                }
                self.data_log.append(data_row)
                self.sample_count += 1
                
                # 每10个样本打印一次（大约每秒）
                if self.sample_count % 10 == 0:
                    print(f"{timestamp:8.2f}s  "
                          f"{self.motor_angles['strike_motor_L1']:8.2f}°  "
                          f"{self.motor_angles['strike_motor_L2']:8.2f}°  "
                          f"{self.motor_angles['strike_motor_R1']:8.2f}°  "
                          f"{self.motor_angles['strike_motor_R2']:8.2f}°")
    
    def save_data(self):
        """保存记录到CSV文件"""
        if not self.data_log:
            print("\n⚠️  没有记录到数据！")
            return
        
        # 生成文件名
        filename = f"motor_angles_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        
        # 写入CSV
        with open(filename, 'w', newline='') as f:
            fieldnames = ['timestamp', 'L1_deg', 'L2_deg', 'R1_deg', 'R2_deg',
                         'L1_rad', 'L2_rad', 'R1_rad', 'R2_rad']
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            
            # 写入表头
            writer.writeheader()
            
            # 写入数据
            for row in self.data_log:
                writer.writerow(row)
        
        print(f"\n{'='*70}")
        print(f"✅ 数据已保存到: {filename}")
        print(f"   共记录 {len(self.data_log)} 个数据点")
        print(f"   持续时间: {self.data_log[-1]['timestamp']:.2f} 秒")
        print(f"   采样频率: ~{len(self.data_log)/self.data_log[-1]['timestamp']:.1f} Hz")
        print(f"{'='*70}")
        
        # 显示统计信息
        print("\n角度范围统计:")
        for motor in ['L1', 'L2', 'R1', 'R2']:
            angles = [row[f'{motor}_deg'] for row in self.data_log]
            print(f"  {motor}: 最小={min(angles):6.2f}°  "
                  f"最大={max(angles):6.2f}°  "
                  f"平均={sum(angles)/len(angles):6.2f}°")


def main():
    rclpy.init()
    recorder = MotorAngleRecorder()
    
    try:
        print("\n提示:")
        print("  - 数据正在实时记录")
        print("  - 您可以手动扳动电机")
        print("  - 按 Ctrl+C 停止记录并保存数据")
        print()
        
        rclpy.spin(recorder)
        
    except KeyboardInterrupt:
        print("\n\n正在停止记录...")
        
    finally:
        recorder.save_data()
        recorder.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

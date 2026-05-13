"""单电机控制示例 - 简化版本"""
import time
from can_driver import CANDriver
from ROS_test.multi_motor_manager import MultiMotorManager

# === 配置 ===
MOTOR_ID = 1
TARGET_ANGLE = 90.0

# === 初始化 ===
driver = CANDriver()
manager = MultiMotorManager(driver)

# 添加单个电机 (使用默认PID参数)
manager.add_motor(MOTOR_ID, TARGET_ANGLE)

# === 主控制循环 ===
try:
    print(f"单电机控制 (ID: {MOTOR_ID}, 目标: {TARGET_ANGLE}°)")
    time.sleep(0.5)
    
    while True:
        manager.send_commands()
        manager.print_status()
        time.sleep(0.02)  # 50Hz显示

except KeyboardInterrupt:
    print("\n停止...")
    manager.stop_all()
    driver.running = False

import time
import sys
import signal
from can_driver import CANDriver
from multi_motor_manager import MultiMotorManager
from pid import PID

# === 全局变量用于信号处理 ===
shutdown_flag = False
# 存储视觉误差 [ID: 误差值]
visual_errors = {1: 0.0, 2: 0.0}
# 存储最后更新时间，防止视觉丢失后电机疯转
last_vision_update = {1: 0.0, 2: 0.0}

def signal_handler(sig, frame):
    """处理 Ctrl+C 信号"""
    global shutdown_flag
    print("\n\n🛑 收到退出信号 (Ctrl+C)...")
    shutdown_flag = True

# 注册信号处理器
signal.signal(signal.SIGINT, signal_handler)

# === ROS集成 (可选) ===
USE_ROS = '--ros' in sys.argv

if USE_ROS:
    try:
        import rospy
        import ros_angle_updater
        
        # 初始化ROS
        try:
            ros_angle_updater.init_ros_node()
        except rospy.exceptions.ROSException as e:
            print(f"\n❌ ROS 初始化失败: {e}")
            print("\n💡 解决方法:")
            print("   1. 先启动 roscore:")
            print("      roscore")
            print("   或")
            print("   2. 使用一键启动脚本:")
            print("      bash start_ros_system.sh")
            print("\n")
            sys.exit(1)
        
        # 定义更新回调
        def on_vision_update(motor_id, error_val):
            """
            当ROS话题更新时触发
            motor_id 1: X轴误差
            motor_id 2: Y轴误差
            """
            visual_errors[motor_id] = error_val
            last_vision_update[motor_id] = time.time() # 记录时间戳
        
        # 注册回调
        ros_angle_updater.register_callback(on_vision_update)
        
        print("[主程序] ROS视觉追踪模式已启用")
        print("[提示] 按 Ctrl+C 可随时退出\n")
        ROS_ENABLED = True
        
    except ImportError as e:
        print(f"\n❌ ROS模块导入失败: {e}")
        print("\n💡 解决方法:")
        print("   bash setup_conda_ros.sh")
        print("   conda deactivate && conda activate base")
        print("\n")
        sys.exit(1)
    except Exception as e:
        print(f"[警告] ROS初始化失败: {e}")
        ROS_ENABLED = False
else:
    ROS_ENABLED = False
    print("[主程序] 非ROS模式")
    print("[提示] 按 Ctrl+C 可随时退出\n")

# === 配置多电机 ===
MOTOR_CONFIGS = [
    {
        'id': 1,
        'target_angle': 150.0,
        'min_angle': 60.0,   # 限制范围
        'max_angle': 270.0,
        'speed_pid': {'kp': 30.0, 'ki': 1.0, 'kd': 0.0, 'i_max': 300, 'out_max': 4000, 'dead_zone': 5},
        'angle_pid': {'kp': 10.0, 'ki': 1.0, 'kd': 0.0, 'i_max': 10, 'out_max': 200, 'dead_zone': 0.5}
    },
    {
        'id': 2,
        'target_angle': 180.0,
        'min_angle': 0.0,    # 限制范围
        'max_angle': 360.0,
        'speed_pid': {'kp': 30.0, 'ki': 1.0, 'kd': 0.0, 'i_max': 300, 'out_max': 4000, 'dead_zone': 5},
        'angle_pid': {'kp': 10.0, 'ki': 1.0, 'kd': 0.0, 'i_max': 10, 'out_max': 200, 'dead_zone': 0.5}
    },
]

# === 初始化 ===
driver = CANDriver()
manager = MultiMotorManager(driver)

# 添加所有配置的电机
for config in MOTOR_CONFIGS:
    manager.add_motor(
        motor_id=config['id'],
        target_angle=config['target_angle'],
        speed_pid_params=config['speed_pid'],
        angle_pid_params=config['angle_pid']
    )

# === 视觉追踪 PID ===
# 输入: 像素误差 (Target=0, Feedback=Error)
# 输出: 角度增量
vis_pid_x = PID(kp=0.001, ki=0.0, kd=0.0001, i_max=0, out_max=3.0, dead_zone=1.0)
vis_pid_y = PID(kp=0.001, ki=0.0, kd=0.0001, i_max=1, out_max=3.0, dead_zone=1.0)

# === 主控制循环 ===
try:
    print("开始多电机双环控制")
    print(f"控制电机数量: {len(MOTOR_CONFIGS)}")
    
    # 等待所有电机初始数据
    print("等待电机反馈...")
    time.sleep(0.5)
    print("开始闭环控制\n")

    loop_count = 0
    
    while not shutdown_flag:
        if ROS_ENABLED and rospy.is_shutdown():
            break
        
        # --- 视觉追踪逻辑 ---
        if ROS_ENABLED:
            current_time = time.time()
            
            # 处理 ID2 (X轴)
            # 1. 超时保护 (0.5秒无数据则停止追踪)
            if current_time - last_vision_update.get(1, 0) > 0.5:
                err_x = 0.0
            else:
                err_x = visual_errors.get(1, 0.0)
            
            # 2. PID计算 (负反馈: 目标0 - 误差)
            delta_x = vis_pid_x.calc(0.0, err_x)
            
            # 处理 ID1 (Y轴)
            if current_time - last_vision_update.get(2, 0) > 0.5:
                err_y = 0.0
            else:
                err_y = visual_errors.get(2, 0.0)
            
            delta_y = vis_pid_y.calc(0.0, err_y)
            

            # 3. 更新目标角度 (累加并限幅)
            if 1 in manager.motors:
                curr = manager.motors[1].target_angle
                new_angle = curr + delta_y
                # 限幅 60-270
                new_angle = max(60.0, min(new_angle, 270.0))
                manager.set_target_angle(1, new_angle)
                
            if 2 in manager.motors:
                curr = manager.motors[2].target_angle
                new_angle = curr + delta_x
                # 限幅 0-360
                new_angle = max(0.0, min(new_angle, 360.0))
                manager.set_target_angle(2, new_angle)
        
        # 发送所有电机的控制命令
        manager.send_commands()
        
        # 每50次循环打印一次状态
        if loop_count % 50 == 0:
            if ROS_ENABLED:
                print(f"[视觉] Err X:{visual_errors.get(1,0):.1f} Y:{visual_errors.get(2,0):.1f}")
            manager.print_status()
            print()
        
        loop_count += 1
        time.sleep(0.005)  # 200Hz

except KeyboardInterrupt:
    print("\n🛑 收到键盘中断...")
except Exception as e:
    print(f"\n❌ 错误: {e}")
    import traceback
    traceback.print_exc()
finally:
    print("\n正在安全停止所有电机...")
    manager.stop_all()
    driver.running = False
    
    if ROS_ENABLED:
        try:
            rospy.signal_shutdown("用户请求退出")
        except:
            pass
    
    print("✅ 程序已安全退出")
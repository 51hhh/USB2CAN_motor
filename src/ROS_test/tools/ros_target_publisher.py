"""简单的ROS测试发布器"""

import rospy
from std_msgs.msg import Float32MultiArray
import math
import signal
import sys

# 退出标志
shutdown_flag = False

def signal_handler(sig, frame):
    """处理 Ctrl+C"""
    global shutdown_flag
    print("\n\n🛑 停止发布器...")
    shutdown_flag = True
    rospy.signal_shutdown("用户退出")
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

def main():
    rospy.init_node('motor_target_test', anonymous=False)
    pub = rospy.Publisher('/motor_targets', Float32MultiArray, queue_size=1)
    rate = rospy.Rate(10)  # 10Hz
    
    t = 0.0
    rospy.loginfo("开始发布目标角度... (按 Ctrl+C 退出)")
    
    try:
        while not rospy.is_shutdown() and not shutdown_flag:
            msg = Float32MultiArray()
            
            # 正弦波运动
            angle1 = 90.0 + 45.0 * math.sin(2 * math.pi * 0.2 * t)
            angle2 = 180.0 + 60.0 * math.cos(2 * math.pi * 0.15 * t)
            
            msg.data = [angle1, angle2]
            pub.publish(msg)
            
            rospy.loginfo(f"目标: M1={angle1:.1f}° M2={angle2:.1f}°")
            
            t += 0.1
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.loginfo("发布器已停止")

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\n退出...")

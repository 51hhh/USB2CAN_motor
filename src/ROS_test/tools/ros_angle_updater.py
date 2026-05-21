"""
独立的ROS节点：订阅目标角度并更新全局变量
通过回调机制与main.py解耦
"""

import rospy
from std_msgs.msg import Float32MultiArray

# === 全局目标角度变量 ===
target_angles = {
    1: 90.0,   # 电机1默认目标
    2: 180.0,  # 电机2默认目标
}

# === 回调函数列表 ===
update_callbacks = []

def register_callback(callback_func):
    """
    注册回调函数
    callback_func 签名: func(motor_id, new_angle)
    """
    update_callbacks.append(callback_func)

def target_callback(msg):
    """ROS话题回调：更新目标角度"""
    try:
        for i, angle in enumerate(msg.data):
            motor_id = i + 1
            
            # 更新全局变量
            target_angles[motor_id] = angle
            
            # 触发所有注册的回调
            for callback in update_callbacks:
                callback(motor_id, angle)
                
            rospy.logdebug(f"电机{motor_id}目标更新: {angle}°")
            
    except Exception as e:
        rospy.logerr(f"更新失败: {e}")

def init_ros_node():
    """初始化ROS节点和订阅器"""
    rospy.init_node('motor_angle_updater', anonymous=False)
    
    rospy.Subscriber(
        '/motor_targets',
        Float32MultiArray,
        target_callback,
        queue_size=1
    )
    
    rospy.loginfo("ROS角度更新器已启动")
    rospy.loginfo("订阅话题: /motor_targets")

def get_target_angle(motor_id):
    """获取指定电机的目标角度"""
    return target_angles.get(motor_id, 0.0)

# === 独立运行模式（调试用） ===
if __name__ == '__main__':
    try:
        init_ros_node()
        
        # 注册测试回调
        def test_callback(motor_id, angle):
            print(f"[测试回调] 电机{motor_id} -> {angle}°")
        
        register_callback(test_callback)
        
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass

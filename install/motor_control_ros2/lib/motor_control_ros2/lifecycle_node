#!/usr/bin/env python3
"""
ROS2 电机控制生命周期节点

负责配置管理和高层控制逻辑。
实时控制由 C++ 节点处理。
"""

import rclpy
from rclpy.lifecycle import Node, State, TransitionCallbackReturn
from rclpy.lifecycle import Publisher, LifecycleState
import yaml
import os

from motor_control_ros2.msg import (
    DJIMotorState, DJIMotorCommand,
    DamiaoMotorState, DamiaoMotorCommand,
    UnitreeMotorState, UnitreeMotorCommand
)
from motor_control_ros2.srv import MotorEnable, MotorSetZero


class MotorLifecycleNode(Node):
    """
    Python 生命周期节点
    
    职责：
    - 加载 YAML 配置
    - 管理节点生命周期
    - 提供高层控制接口
    """
    
    def __init__(self):
        super().__init__('motor_lifecycle_node')
        
        # 声明参数
        self.declare_parameter('config_file', '')
        
        # 配置数据
        self.config = None
        self.motors = {}
        
        self.get_logger().info('电机生命周期节点已创建')
    
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """配置回调"""
        self.get_logger().info('正在配置电机控制节点...')
        
        # 加载配置文件
        config_file = self.get_parameter('config_file').value
        if not config_file:
            self.get_logger().error('未指定配置文件！')
            return TransitionCallbackReturn.FAILURE
        
        # 检查文件是否存在
        if not os.path.exists(config_file):
            self.get_logger().error(f'配置文件不存在: {config_file}')
            return TransitionCallbackReturn.FAILURE
        
        # 加载 YAML
        try:
            with open(config_file, 'r', encoding='utf-8') as f:
                self.config = yaml.safe_load(f)
            self.get_logger().info(f'成功加载配置文件: {config_file}')
        except Exception as e:
            self.get_logger().error(f'加载配置文件失败: {e}')
            return TransitionCallbackReturn.FAILURE
        
        # 解析电机配置
        if 'motors' not in self.config:
            self.get_logger().error('配置文件中没有 motors 字段')
            return TransitionCallbackReturn.FAILURE
        
        self.motors = self.config['motors']
        self.get_logger().info(f'配置了 {len(self.motors)} 个电机')
        
        # 创建发布者和订阅者
        self._create_publishers()
        self._create_subscribers()
        self._create_services()
        
        self.get_logger().info('配置完成')
        return TransitionCallbackReturn.SUCCESS
    
    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """激活回调"""
        self.get_logger().info('正在激活电机控制...')
        
        # 激活发布者
        # (ROS2 生命周期发布者会自动激活)
        
        self.get_logger().info('电机控制已激活')
        return TransitionCallbackReturn.SUCCESS
    
    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """停用回调"""
        self.get_logger().info('正在停用电机控制...')
        
        # 发送零命令到所有电机（安全停止）
        # TODO: 实现
        
        self.get_logger().info('电机控制已停用')
        return TransitionCallbackReturn.SUCCESS
    
    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """清理回调"""
        self.get_logger().info('正在清理资源...')
        
        # 销毁发布者和订阅者
        # (ROS2 会自动处理)
        
        self.get_logger().info('清理完成')
        return TransitionCallbackReturn.SUCCESS
    
    def _create_publishers(self):
        """创建发布者"""
        # 这里可以根据配置创建不同类型的发布者
        # 示例：为每个电机创建命令发布者
        pass
    
    def _create_subscribers(self):
        """创建订阅者"""
        # 订阅电机状态
        self.create_subscription(
            DJIMotorState,
            'dji_motor_states',
            self._dji_state_callback,
            10
        )
        
        self.create_subscription(
            DamiaoMotorState,
            'damiao_motor_states',
            self._damiao_state_callback,
            10
        )
    
    def _create_services(self):
        """创建服务"""
        self.create_service(
            MotorEnable,
            'motor_enable',
            self._enable_callback
        )
        
        self.create_service(
            MotorSetZero,
            'motor_set_zero',
            self._set_zero_callback
        )
    
    def _dji_state_callback(self, msg):
        """DJI 电机状态回调"""
        # 可以在这里添加状态监控逻辑
        pass
    
    def _damiao_state_callback(self, msg):
        """达妙电机状态回调"""
        pass
    
    def _enable_callback(self, request, response):
        """电机使能服务回调"""
        self.get_logger().info(f'收到使能请求: {request.motor_names}, enable={request.enable}')
        
        # TODO: 实现使能逻辑
        response.success = True
        response.message = "电机使能命令已发送"
        return response
    
    def _set_zero_callback(self, request, response):
        """设置零点服务回调"""
        self.get_logger().info(f'收到设置零点请求: {request.motor_names}')
        
        # TODO: 实现设置零点逻辑
        response.success = True
        response.message = "零点设置命令已发送"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MotorLifecycleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

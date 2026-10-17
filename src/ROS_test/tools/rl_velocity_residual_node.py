#!/usr/bin/env python3
"""Fail-closed ROS 2 bridge: classic chassis command + bounded RL residual."""
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray

from motor_control_ros2.msg import ChassisState
from rl_velocity_contract import Limits, mix


class RlVelocityResidualNode(Node):
    def __init__(self):
        super().__init__("rl_velocity_residual_node")
        for name,default in (
            ("max_vx_m_s",0.),("max_vy_m_s",0.),("max_wz_rad_s",0.),
            ("max_ax_m_s2",0.),("max_ay_m_s2",0.),("max_awz_rad_s2",0.),
            ("residual_fraction",.05),("publish_rate_hz",50.),
            ("classic_watchdog_s",.2),("residual_watchdog_s",.1),
            ("health_watchdog_s",.2),("estop_watchdog_s",.2)):
            self.declare_parameter(name,default)
        for name,default in (
            ("classic_topic","/cmd_vel_classic"),
            ("residual_topic","/rl/swerve_residual_normalized"),
            ("chassis_state_topic","/chassis_state"),
            ("estop_topic","/emergency_stop"),
            ("output_topic","/cmd_vel_remote")):
            self.declare_parameter(name,default)
        p=lambda name:self.get_parameter(name).value
        self.limits=Limits((p("max_vx_m_s"),p("max_vy_m_s"),p("max_wz_rad_s")),
                           (p("max_ax_m_s2"),p("max_ay_m_s2"),p("max_awz_rad_s2")),
                           p("residual_fraction"))
        self.rate=float(p("publish_rate_hz"))
        self.watchdogs={key:float(p(key)) for key in
                        ("classic_watchdog_s","residual_watchdog_s",
                         "health_watchdog_s","estop_watchdog_s")}
        if self.rate<=0 or min(self.watchdogs.values())<=0:
            raise ValueError("rate and watchdogs must be positive")
        self.classic=(0.,0.,0.);self.action=(0.,0.,0.);self.previous=(0.,0.,0.)
        self.stamps={"classic":None,"residual":None,"health":None,"estop":None}
        self.health_ok=False;self.estop=True
        self.create_subscription(Twist,p("classic_topic"),self.on_classic,10)
        self.create_subscription(Float32MultiArray,p("residual_topic"),self.on_residual,10)
        self.create_subscription(ChassisState,p("chassis_state_topic"),self.on_health,10)
        self.create_subscription(Bool,p("estop_topic"),self.on_estop,10)
        self.publisher=self.create_publisher(Twist,p("output_topic"),10)
        self.create_timer(1/self.rate,self.publish)

    def now(self):return self.get_clock().now()
    def fresh(self,name):
        stamp=self.stamps[name]
        return stamp is not None and (self.now()-stamp).nanoseconds*1e-9 <= self.watchdogs[name+"_watchdog_s"]
    def on_classic(self,msg):
        values=(msg.linear.x,msg.linear.y,msg.angular.z)
        if all(math.isfinite(value) for value in values):
            self.classic=values;self.stamps["classic"]=self.now()
    def on_residual(self,msg):
        values=tuple(msg.data)
        if len(values)==3 and all(math.isfinite(value) and abs(value)<=1 for value in values):
            self.action=values;self.stamps["residual"]=self.now()
        else:
            self.stamps["residual"]=None
            self.get_logger().error("拒绝非法RL残差，回退经典控制")
    def on_health(self,msg):
        self.health_ok=bool(msg.all_motors_healthy) and not bool(msg.estop_active)
        self.stamps["health"]=self.now()
    def on_estop(self,msg):
        self.estop=bool(msg.data);self.stamps["estop"]=self.now()
    def publish(self):
        command,_,_=mix(self.classic,self.action,self.limits,self.previous,1/self.rate,
                        classic_fresh=self.fresh("classic"),
                        residual_fresh=self.fresh("residual"),
                        health_ok=self.health_ok and self.fresh("health"),
                        estop=self.estop or not self.fresh("estop"))
        self.previous=command
        msg=Twist();msg.linear.x,msg.linear.y,msg.angular.z=command
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args);node=RlVelocityResidualNode()
    try:rclpy.spin(node)
    finally:node.destroy_node();rclpy.shutdown()


if __name__=="__main__":main()

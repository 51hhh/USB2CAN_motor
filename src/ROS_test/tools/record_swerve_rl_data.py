#!/usr/bin/env python3
"""Record synchronized real four-swerve data in the RSI G1 CSV contract."""
import csv
import hashlib
import json
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

from motor_control_ros2.msg import ChassisState
from swerve_recording_contract import HEADER,build_row,stamp_s


class RealSwerveRecorder(Node):
    def __init__(self):
        super().__init__("real_swerve_recorder")
        for name,default in (("output_path",""),("session_id",""),("operator",""),
                             ("controller_version",""),("firmware_version",""),
                             ("chassis_manifest_sha256",""),
                             ("expected_odom_frame","odom"),("expected_base_frame","base_link"),
                             ("chassis_state_topic","/chassis_state"),("odom_topic","/odom")):
            self.declare_parameter(name,default)
        for name,default in (("confirm_real_robot",False),("duration_s",30.),
                             ("wheel_radius_m",0.),("max_sync_error_s",.03)):
            self.declare_parameter(name,default)
        p=lambda name:self.get_parameter(name).value
        if not p("confirm_real_robot"):
            raise ValueError("必须显式设置 confirm_real_robot:=true")
        required=("session_id","operator","output_path","controller_version",
                  "firmware_version","chassis_manifest_sha256")
        if any(not p(name) for name in required):
            raise ValueError("session/operator/output/controller/firmware/chassis SHA256 均为必填")
        chassis_sha=str(p("chassis_manifest_sha256"))
        if len(chassis_sha)!=64 or any(char not in "0123456789abcdef" for char in chassis_sha):
            raise ValueError("chassis_manifest_sha256 必须是64位小写SHA256")
        self.duration=float(p("duration_s"));self.radius=float(p("wheel_radius_m"))
        self.max_sync=float(p("max_sync_error_s"))
        if self.duration<30 or self.radius<=0 or self.max_sync<=0:
            raise ValueError("duration_s>=30，wheel_radius_m>0，max_sync_error_s>0")
        self.target=Path(p("output_path")).expanduser().resolve()
        self.partial=self.target.with_suffix(self.target.suffix+".partial")
        self.metadata_path=self.target.with_suffix(self.target.suffix+".metadata.json")
        if self.target.exists() or self.partial.exists() or self.metadata_path.exists():
            raise FileExistsError(self.target)
        self.target.parent.mkdir(parents=True,exist_ok=True)
        self.stream=self.partial.open("x",newline="");self.writer=csv.DictWriter(self.stream,fieldnames=HEADER)
        self.writer.writeheader();self.stream.flush()
        self.session_id=str(p("session_id"));self.operator=str(p("operator"))
        self.controller_version=str(p("controller_version"))
        self.firmware_version=str(p("firmware_version"));self.chassis_sha=chassis_sha
        self.odom_frame=str(p("expected_odom_frame"));self.base_frame=str(p("expected_base_frame"))
        self.odom=None;self.first_stamp=None;self.last_stamp=None;self.rows=0;self.finalized=False
        self.create_subscription(Odometry,p("odom_topic"),self.on_odom,20)
        self.create_subscription(ChassisState,p("chassis_state_topic"),self.on_state,20)

    def on_odom(self,msg):self.odom=msg
    def on_state(self,state):
        if self.finalized or self.odom is None:return
        if (self.odom.header.frame_id!=self.odom_frame or
                self.odom.child_frame_id!=self.base_frame or
                state.header.frame_id!=self.base_frame):
            return
        t=stamp_s(state.header.stamp);odom_t=stamp_s(self.odom.header.stamp)
        if self.last_stamp is not None and t<=self.last_stamp:return
        if abs(t-odom_t)>self.max_sync:return
        try:row=build_row(state,self.odom,self.radius)
        except ValueError:return
        self.writer.writerow(row);self.stream.flush();self.rows+=1
        self.first_stamp=t if self.first_stamp is None else self.first_stamp;self.last_stamp=t
        if self.last_stamp-self.first_stamp>=self.duration:self.finalize()

    def finalize(self):
        if self.finalized:return
        self.stream.flush();self.stream.close();self.partial.replace(self.target)
        digest=hashlib.sha256(self.target.read_bytes()).hexdigest()
        metadata={"schema":"swerve.real_raw_session.v1","source":"real_robot",
                  "session_id":self.session_id,"operator":self.operator,"csv":self.target.name,
                  "controller_version":self.controller_version,
                  "firmware_version":self.firmware_version,
                  "chassis_manifest_sha256":self.chassis_sha,
                  "frames":{"odom":self.odom_frame,"base":self.base_frame},
                  "sha256":digest,"samples":self.rows,"duration_s":self.last_stamp-self.first_stamp,
                  "units":{"command":"m_s,m_s,rad_s","pose":"m,m,rad",
                           "twist":"m_s,m_s,rad_s","steer":"rad","wheel":"rad_s",
                           "current":"raw_dji_current"},
                  "synchronization":{"clock":"ROS header stamps","max_error_s":self.max_sync}}
        with self.metadata_path.open("x") as stream:
            stream.write(json.dumps(metadata,indent=2,ensure_ascii=False)+"\n")
        self.finalized=True
        self.get_logger().info(f"录制完成: {self.target} rows={self.rows} sha256={digest}")
        rclpy.shutdown()

    def abort(self):
        if not self.stream.closed:self.stream.close()


def main(args=None):
    rclpy.init(args=args);node=RealSwerveRecorder()
    try:rclpy.spin(node)
    finally:
        if not node.finalized:node.abort()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=="__main__":main()

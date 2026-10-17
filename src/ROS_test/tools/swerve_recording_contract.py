"""Pure data mapping for the real-swerve G1 CSV recorder."""
from __future__ import annotations

import math


HEADER=("timestamp_s","requested_cmd_vx","requested_cmd_vy","requested_cmd_wz",
        "cmd_vx","cmd_vy","cmd_wz","all_motors_online","all_motors_healthy","command_timed_out",
        "estop_active","safety_stop","pose_x","pose_y","pose_yaw",
        "twist_vx","twist_vy","twist_wz","steer_fl","steer_fr","steer_rl","steer_rr",
        "wheel_fl","wheel_fr","wheel_rl","wheel_rr","sat_steer_fl","sat_steer_fr",
        "sat_steer_rl","sat_steer_rr","sat_drive_fl","sat_drive_fr","sat_drive_rl",
        "sat_drive_rr","fault_steer_fl","fault_steer_fr","fault_steer_rl","fault_steer_rr",
        "fault_drive_fl","fault_drive_fr","fault_drive_rl","fault_drive_rr",
        "current_steer_fl","current_steer_fr","current_steer_rl","current_steer_rr",
        "current_drive_fl","current_drive_fr","current_drive_rl","current_drive_rr")


def stamp_s(stamp):return float(stamp.sec)+float(stamp.nanosec)*1e-9
def yaw_from_quaternion(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


def build_row(state,odom,wheel_radius_m):
    if not math.isfinite(wheel_radius_m) or wheel_radius_m<=0:
        raise ValueError("wheel_radius_m must be positive and finite")
    modules=(state.fl_wheel,state.fr_wheel,state.rl_wheel,state.rr_wheel)
    pose=odom.pose.pose;twist=odom.twist.twist
    row={"timestamp_s":stamp_s(state.header.stamp),
         "requested_cmd_vx":state.requested_linear_x,
         "requested_cmd_vy":state.requested_linear_y,
         "requested_cmd_wz":state.requested_angular_z,
         "cmd_vx":state.command_linear_x,"cmd_vy":state.command_linear_y,
         "cmd_wz":state.command_angular_z,"all_motors_online":int(state.all_motors_online),
         "all_motors_healthy":int(state.all_motors_healthy),
         "command_timed_out":int(state.command_timed_out),
         "estop_active":int(state.estop_active),"safety_stop":int(state.safety_stop),
         "pose_x":pose.position.x,"pose_y":pose.position.y,
         "pose_yaw":yaw_from_quaternion(pose.orientation),"twist_vx":twist.linear.x,
         "twist_vy":twist.linear.y,"twist_wz":twist.angular.z}
    for tag,module in zip(("fl","fr","rl","rr"),modules):
        row["steer_"+tag]=math.radians(module.steer_angle)
        row["wheel_"+tag]=module.drive_velocity/wheel_radius_m
        row["sat_steer_"+tag]=int(module.steer_saturated)
        row["sat_drive_"+tag]=int(module.drive_saturated)
        row["fault_steer_"+tag]=int(module.steer_fault)
        row["fault_drive_"+tag]=int(module.drive_fault)
        row["current_steer_"+tag]=module.steer_current
        row["current_drive_"+tag]=module.drive_current
    if set(row)!=set(HEADER) or not all(math.isfinite(float(value)) for value in row.values()):
        raise ValueError("recording row is incomplete or non-finite")
    return row

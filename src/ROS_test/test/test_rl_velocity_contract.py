import math
import sys
from pathlib import Path
from types import SimpleNamespace as NS
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"tools"))
from rl_velocity_contract import Limits,mix
from swerve_recording_contract import HEADER,build_row


class ContractTest(unittest.TestCase):
    def setUp(self):self.limits=Limits((2,1,4),(1,2,3),.05)
    def test_five_percent_residual_and_slew(self):
        command,mode,saturated=mix((.5,.2,.5),(1,-1,.5),self.limits,dt=.1)
        self.assertEqual(mode,"classic_plus_residual");self.assertTrue(saturated)
        for actual,expected in zip(command,(.1,.15,.3)):self.assertAlmostEqual(actual,expected)
    def test_rl_timeout_preserves_classic(self):
        command,mode,_=mix((.2,.1,.3),(math.nan,),self.limits,
                           previous=(.2,.1,.3),residual_fresh=False)
        self.assertEqual(command,(.2,.1,.3));self.assertEqual(mode,"classic_only_rl_timeout")
    def test_safety_stops_bypass_untrusted_input(self):
        for kwargs in ({"estop":True},{"health_ok":False},{"classic_fresh":False}):
            self.assertEqual(mix((math.nan,),(),self.limits,**kwargs)[0],(0.,0.,0.))
    def test_action_range_is_enforced(self):
        with self.assertRaises(ValueError):mix((0,0,0),(1.01,0,0),self.limits)

    def test_recorder_contains_rsi_required_and_safety_columns(self):
        header=set(HEADER)
        required={"timestamp_s","cmd_vx","cmd_vy","cmd_wz","pose_x","pose_y","pose_yaw",
                  "twist_vx","twist_vy","twist_wz","all_motors_online","all_motors_healthy",
                  "command_timed_out","estop_active","safety_stop"}
        for prefix in ("steer","wheel"):
            required.update(prefix+"_"+module for module in ("fl","fr","rl","rr"))
        for signal in ("sat_steer","sat_drive","fault_steer","fault_drive",
                       "current_steer","current_drive"):
            required.update(signal+"_"+module for module in ("fl","fr","rl","rr"))
        self.assertFalse(required-header)

    def test_recording_row_converts_degrees_and_linear_speed(self):
        module=lambda angle,speed:NS(steer_angle=angle,drive_velocity=speed,
            steer_saturated=False,drive_saturated=True,steer_fault=False,drive_fault=False,
            steer_current=10,drive_current=20)
        state=NS(header=NS(stamp=NS(sec=4,nanosec=500_000_000)),requested_linear_x=1,
            requested_linear_y=2,requested_angular_z=3,command_linear_x=.1,
            command_linear_y=.2,command_angular_z=.3,all_motors_online=True,
            all_motors_healthy=True,command_timed_out=False,estop_active=False,safety_stop=False,
            fl_wheel=module(180,.55),fr_wheel=module(0,0),rl_wheel=module(0,0),rr_wheel=module(0,0))
        odom=NS(pose=NS(pose=NS(position=NS(x=1,y=2),orientation=NS(w=1,x=0,y=0,z=0))),
                twist=NS(twist=NS(linear=NS(x=.1,y=.2),angular=NS(z=.3))))
        row=build_row(state,odom,.055)
        self.assertEqual(set(row),set(HEADER));self.assertAlmostEqual(row["timestamp_s"],4.5)
        self.assertAlmostEqual(row["steer_fl"],math.pi);self.assertAlmostEqual(row["wheel_fl"],10)


if __name__=="__main__":unittest.main()

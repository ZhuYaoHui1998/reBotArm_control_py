#!/usr/bin/env python3
"""机械臂零点校准 + 实时角度监控（失能）/ Zero all joints and stream angles (disabled holding)."""
import time
from pathlib import Path

import numpy as np
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from reBotArm_control_py.actuator import RobotArm


def mit_controller(ref, dt):
    ref.mit(np.zeros(ref.num_joints),
            kp=np.zeros(ref.num_joints),
            kd=np.zeros(ref.num_joints),
            tau=np.zeros(ref.num_joints))


arm = RobotArm()
arm.connect()
print("--- connected / 连接成功 ---")
arm.set_zero()
print("--- zero set / 零点已设置 ---\n")

print("\n--- live angles (deg), Ctrl+C to quit / 实时角度 Ctrl+C 退出 ---\n")
arm.start_control_loop(mit_controller)
try:
    while True:
        positions = arm.get_positions()
        row = "  ".join(f"{p*180/np.pi:+.2f}" for p in positions)
        print(f"\r{row}  ", end="", flush=True)
        time.sleep(0.002)
except (KeyboardInterrupt, EOFError):
    pass
finally:
    arm.disconnect()

#!/usr/bin/env python3
"""IK 仿真 + MeshCat / Interactive IK with visualization.

用法 / Usage:
    uv run python example/sim/ik_sim.py

控制 / Controls:
    输入目标位置 x y z (米) / target position x y z (m)
    可选: 姿态 roll pitch yaw (弧度) / optional RPY (rad)
    例 / ex: 0.25 0.0 0.15          (仅位置 / pos only)
    例 / ex: 0.25 0.0 0.15 0 0 0    (位置+姿态 / pose)
    q / quit / exit: 退出 / quit
"""

import sys
import signal
import time
from pathlib import Path

import numpy as np
import pinocchio as pin

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from reBotArm_control_py.kinematics.inverse_kinematics import compute_ik
from example.sim.visualizer import Visualizer

should_exit = False


def signal_handler(sig, frame):
    global should_exit
    should_exit = True


def main():
    signal.signal(signal.SIGINT, signal_handler)

    print("加载可视化器... / Loading visualizer...")
    viz = Visualizer()

    viz.neutral()

    print("MeshCat 已打开. 输入目标位姿: / MeshCat ready. Target pose:")
    print("  x y z                      (仅位置，米 / m, pos only)")
    print("  x y z roll pitch yaw       (位置+姿态，弧度 / m + rad)")
    print("  q/quit/exit: 退出 / quit\n")

    while not should_exit:
        time.sleep(0.01)

        try:
            line = input("目标位姿 / target pose > ").strip().lower()
        except EOFError:
            break

        if line in ("q", "quit", "exit", ""):
            break

        try:
            vals = [float(x) for x in line.split()]
            if len(vals) not in (3, 6):
                print("需要 3 或 6 个值 / need 3 or 6 numbers\n")
                continue
        except ValueError:
            print("无效输入 / invalid input\n")
            continue

        target_pos = np.array(vals[:3])  # 位置 / position
        target_rot = None
        if len(vals) == 6:
            r, p, y = vals[3], vals[4], vals[5]
            target_rot = pin.rpy.rpyToMatrix(r, p, y)  # 姿态 / orientation

        result = compute_ik(None, target_pos, target_rot)

        viz.update(result.q)
        status = "收敛 converged" if result.success else "未收敛 not converged"
        print(f"  [{status}] 迭代 / iters={result.iterations} 误差 / err={result.error:.2e}m")
        print(f"  关节角度(deg) / joints: {np.degrees(result.q)}\n")


if __name__ == "__main__":
    main()

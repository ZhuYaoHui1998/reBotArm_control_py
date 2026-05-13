#!/usr/bin/env python3
"""FK 仿真 + MeshCat / Interactive FK with live visualization.

用法 / Usage:
    python example/sim/fk_sim.py

控制 / Controls:
    输入 6 个关节角度（度），空格分隔 / six joint angles (deg)
    例 / ex: 0 0 0 0 0 0
    例 / ex: 45 -30 15 -60 90 180
    q / quit / exit: 退出 / quit
"""

import sys
import signal
import time
from pathlib import Path

import numpy as np
import pinocchio as pin

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from reBotArm_control_py.kinematics import compute_fk
from example.sim.visualizer import Visualizer

should_exit = False


def signal_handler(sig, frame):
    global should_exit
    should_exit = True
    print("\n退出. / Bye.")


def main():
    signal.signal(signal.SIGINT, signal_handler)

    print("加载可视化器... / Loading visualizer...")
    viz = Visualizer()
    q = np.zeros(viz.nq)
    viz.update(q)

    print("MeshCat 已打开. 输入 6 个关节角度（度）: / MeshCat ready. Enter 6 joint angles (deg):")
    print("  q/quit/exit: 退出 / quit\n")

    while not should_exit:
        time.sleep(0.01)

        try:
            line = input("关节角度 / joint angles (deg) > ").strip().lower()
        except EOFError:
            break

        if line in ("q", "quit", "exit", ""):
            break

        try:
            q_deg = [float(x) for x in line.split()]
            if len(q_deg) != viz.nq:
                print(f"需要 {viz.nq} 个值 / need {viz.nq} values\n")
                continue
        except ValueError:
            print("无效输入 / invalid input\n")
            continue

        q = np.radians(q_deg)
        viz.update(q)

        pos, rot, _ = compute_fk(viz.model, q)
        euler = np.degrees(pin.rpy.matrixToRpy(rot))
        print(f"  末端位置 / EE pos: [{pos[0]:+.4f}, {pos[1]:+.4f}, {pos[2]:+.4f}] m")
        print(f"  末端姿态 / EE rpy: [{euler[0]:+.2f}, {euler[1]:+.2f}, {euler[2]:+.2f}] deg\n")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""reBotArm 正运动学测试 / Forward kinematics console test.

用法 / Usage:
  python example/5_fk_test.py

输入 / Input: six joint angles in degrees, space-separated
输出 / Output: end-effector position (x,y,z) in meters,
      rotation matrix (3×3),
      Euler roll/pitch/yaw in degrees
"""

import sys
import numpy as np
import pinocchio as pin

sys.path.insert(0, str(__file__).rsplit("/", 2)[0])

from reBotArm_control_py.kinematics import (
    load_robot_model,
    compute_fk,
    get_joint_names,
)

# ----------------------------------------------------------------------
# 打印 / Printing helpers
# ----------------------------------------------------------------------
def print_welcome(model, joint_names) -> None:
    print("=" * 52)
    print("  reBotArm 正运动学测试 / Forward kinematics (FK) test")
    print("=" * 52)
    print(f"  机器人 / robot  : {model.name}")
    print(f"  关节 / joints    : {joint_names}")
    print(f"  nq = {model.nq}, nv = {model.nv}")
    print()
    print("  输入 6 个关节角度（度），空格分隔。/ Enter 6 joint angles (deg), space-separated.")
    print("  示例 / ex:  0 0 0 0 0 0")
    print("  示例 / ex:  45 -30 15 -60 90 180")
    print("-" * 52)
    print("> ", end="", flush=True)

def print_result(q_deg, position, rotation, euler_deg) -> None:
    print()
    print("=" * 52)
    print("  结果 / Result")
    print("=" * 52)
    print(f"  关节角度 (度) / joint angles (deg): {q_deg}")
    print()
    print(f"  末端位置 (m) / EE position (m):")
    print(f"    X = {position[0]:+.6f}")
    print(f"    Y = {position[1]:+.6f}")
    print(f"    Z = {position[2]:+.6f}")
    print()
    print(f"  旋转矩阵 (R_world^end) / rotation matrix:")
    for row in rotation:
        print(f"    [{row[0]:+.6f}  {row[1]:+.6f}  {row[2]:+.6f}]")
    print()
    print(f"  欧拉角 XYZ (横滚, 俯仰, 偏航) [度] / Euler XYZ roll,pitch,yaw [deg]:")
    print(f"    横滚 roll  = {euler_deg[0]:+.4f}")
    print(f"    俯仰 pitch = {euler_deg[1]:+.4f}")
    print(f"    偏航 yaw   = {euler_deg[2]:+.4f}")

def parse_joint_input(line: str) -> np.ndarray:
    tokens = line.split()
    if len(tokens) != 6:
        print(f"错误 / error: 需要 6 个值，输入了 {len(tokens)} 个 / need 6 values, got {len(tokens)}")
        sys.exit(1)
    try:
        q_deg = [float(x) for x in tokens]
    except ValueError as e:
        print(f"错误 / error: 无法解析数字 — {e} / cannot parse number: {e}")
        sys.exit(1)
    return np.radians(q_deg)


# ----------------------------------------------------------------------
# 核心算法 / Core FK
# ----------------------------------------------------------------------
def compute_fk_from_deg(model, q_deg: list) -> tuple:
    q_rad = np.radians(q_deg)
    position, rotation, homogeneous = compute_fk(model, q_rad)
    euler_deg = np.degrees(pin.rpy.matrixToRpy(rotation))
    return position, rotation, euler_deg

# ----------------------------------------------------------------------
# main
# ----------------------------------------------------------------------
def main() -> None:
    model = load_robot_model()
    joint_names = get_joint_names(model)

    print_welcome(model, joint_names)

    try:
        line = input().strip()
    except EOFError:
        print("无输入，退出。/ No input, exiting.")
        return

    q_rad = parse_joint_input(line)
    q_deg = np.degrees(q_rad)

    position, rotation, euler_deg = compute_fk_from_deg(model, q_deg)

    print_result(q_deg, position, rotation, euler_deg)

if __name__ == "__main__":
    main()

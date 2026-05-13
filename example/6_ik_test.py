#!/usr/bin/env python3
"""reBotArm 逆运动学测试 / Inverse kinematics console test.

用法 / Usage:
  python example/6_ik_test.py

输入 / Input: target EE position (x y z) in meters;
      optional orientation (roll pitch yaw) in degrees
输出 / Output: six joint angles in degrees + convergence info
"""

import sys
import numpy as np
import pinocchio as pin

sys.path.insert(0, str(__file__).rsplit("/", 2)[0])

from reBotArm_control_py.kinematics import (
    load_robot_model,
    compute_ik,
    get_joint_names,
)


# ----------------------------------------------------------------------
# 打印 / UI 相关
# ----------------------------------------------------------------------

def print_welcome(model, joint_names) -> None:
    print("=" * 52)
    print("  reBotArm 逆运动学测试 / Inverse kinematics (IK) test")
    print("=" * 52)
    print(f"  机器人 / robot: {model.name}")
    print(f"  关节 / joints  : {joint_names}")
    print()
    print("  输入末端期望位姿: / Desired EE pose:")
    print("    <x> <y> <z>                       (仅位置，米) / position only, m")
    print("    <x> <y> <z> <roll> <pitch> <yaw>    (位置+姿态，度) / pos+orient, deg")
    print()
    print("  示例 / examples:")
    print("    0.25 0.0 0.15                      (仅位置 / position only)")
    print("    0.25 0.0 0.15 0 0 0                (位置+姿态 / pos+orient)")
    print("-" * 52)
    print("> ", end="", flush=True)


def print_result(result, target_pos, target_rot, joint_names) -> None:
    print()
    print("=" * 52)
    print("  结果 / Result")
    print("=" * 52)
    print(f"  目标末端位置 / target pos: [{target_pos[0]:+.4f}, {target_pos[1]:+.4f}, {target_pos[2]:+.4f}] m")
    if target_rot is not None:
        euler_in = np.degrees(pin.rpy.matrixToRpy(target_rot))
        print(f"  目标末端姿态 / target orient: [{euler_in[0]:+.2f}, {euler_in[1]:+.2f}, {euler_in[2]:+.2f}] deg")
    print()
    print(f"  收敛状态 / converged: {'是 yes' if result.converged else '否 no'}")
    print(f"  迭代次数 / iterations: {result.iterations}")
    print(f"  位置误差 / pos err: {result.residual_trans:.2e} m")
    print(f"  姿态误差 / rot err: {result.residual_rot:.2e} rad")
    print()
    print(f"  关节角度 (度) / joint angles (deg):")
    for name, deg, rad in zip(joint_names, np.degrees(result.q), result.q):
        print(f"    {name:10s} = {deg:+8.4f} deg  ({rad:+.4f} rad)")


def parse_pose_input(line: str) -> tuple:
    tokens = line.split()
    if len(tokens) not in (3, 6):
        print(f"错误 / error: 需要 3 或 6 个值，输入了 {len(tokens)} 个 / need 3 or 6 values, got {len(tokens)}")
        sys.exit(1)
    try:
        vals = [float(x) for x in tokens]
    except ValueError as e:
        print(f"错误 / error: 无法解析数字 — {e} / cannot parse: {e}")
        sys.exit(1)

    target_pos = np.array(vals[:3])
    target_rot = None
    if len(vals) == 6:
        roll, pitch, yaw = np.radians(vals[3:6])
        target_rot = pin.rpy.rpyToMatrix(roll, pitch, yaw)
    return target_pos, target_rot


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

    target_pos, target_rot = parse_pose_input(line)

    q_init = np.zeros(model.nq)
    result = compute_ik(
        model=model,
        q_init=q_init,
        target_position=target_pos,
        target_rotation=target_rot,
        max_iter=2000,
        damping=0.01,
    )

    print_result(result, target_pos, target_rot, joint_names)


if __name__ == "__main__":
    main()

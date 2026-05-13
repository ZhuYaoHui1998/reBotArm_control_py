#!/usr/bin/env python3
"""重力补偿控制演示 / Gravity-compensation demo (MIT + feedforward g(q)).

使用 Pinocchio 计算广义重力 g(q)，
通过 MIT 模式的前馈力矩直接补偿重力，使机械臂可以在任意姿态下
"漂浮"，即松开后不会因自重坠落。
/ Uses Pinocchio \\(g(q)\\) as MIT feedforward so the arm can be moved gently without sagging.

控制律（MIT 位置闭环 + 重力前馈）：
    tau = g(q)          — 重力前馈 / gravity feedforward
    pos = 当前电机位置   — 关节位置目标跟随当前位置 / pos target tracks measured q
    kp   = 0,  kd = 1.0   — 所有电机统一刚度/阻尼 / uniform low stiffness, damping

终端持续打印每个关节的期望力矩（N·m）。
/ Console prints desired gravity torque per joint (N·m).
"""
import signal
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from reBotArm_control_py.actuator import RobotArm
from reBotArm_control_py.dynamics import (
    load_dynamics_model,
    compute_generalized_gravity,
    get_default_gravity,
)


# --------------------------------------------------------------------------- #
# 全局控制标志 / Global run flag
# --------------------------------------------------------------------------- #

_running = True


def _sigint_handler(signum, frame):
    global _running
    print("\n[gravity_comp] Ctrl+C, stopping... / 收到 Ctrl+C，准备停止...")
    _running = False


signal.signal(signal.SIGINT, _sigint_handler)


# --------------------------------------------------------------------------- #
# 控制回调（每周期调用一次，由 RobotArm 控制循环驱动）
# / Per-tick callback driven by RobotArm control loop
# --------------------------------------------------------------------------- #

def gravity_compensation_controller(arm: RobotArm, dt: float) -> None:
    """重力补偿控制回调 / Gravity-compensation callback.

    读取当前关节位置 → Pinocchio 计算 g(q) → MIT 前馈力矩。
    / Read q → compute g(q) → MIT with feedforward torque.
    """
    # 1. 读取当前关节位置 / read joint positions
    q = arm.get_positions()          # shape=(6,), 单位: rad / rad

    # 2. Pinocchio 计算广义重力向量 / generalized gravity from Pinocchio
    tau_g = compute_generalized_gravity(q=q)   # shape=(6,), 单位: N·m / N·m

    # 3. MIT 前馈: 位置目标跟随当前电机位置，kp=0, kd=1，重力补偿
    #    / MIT: track current q, kp/kd as configured, feedforward tau_g
    arm.mit(
        pos=q,
        vel=np.zeros(arm.num_joints),
        kp=np.full(arm.num_joints, 0.0),
        kd=np.full(arm.num_joints, 1.0),
        tau=tau_g,
        request_feedback=True,
    )

    # 4. 终端打印（每隔 ~20 个周期打印一次，避免刷屏）
    #    / Throttle prints (~ every 20 ticks) to avoid spam
    gravity_compensation_controller._counter += 1
    if gravity_compensation_controller._counter % 20 == 0:
        print(
            f"[{gravity_compensation_controller._counter:4d}] "
            f"tau_g = " + "  ".join(f"{t:+.3f}" for t in tau_g) + "  N·m"
        )


gravity_compensation_controller._counter = 0


# --------------------------------------------------------------------------- #
# 主程序
# --------------------------------------------------------------------------- #

def main() -> None:
    print("=" * 60)
    print("  reBotArm 重力补偿演示 / Gravity-compensation demo")
    print("  预计行为: 机械臂维持位置不动，可以手动掰动至任何位置）/ Arm should float; you can guide it by hand.")
    print("  Ctrl+C 停止并断开连接 / Ctrl+C: stop loop and disconnect")
    print("=" * 60)

    # 动力学模型初始化 / load dynamics model
    model = load_dynamics_model()
    g_vec = get_default_gravity()
    print(f"\n[模型 / model] nq={model.nq}, nv={model.nv}")
    print(f"[重力 / gravity] {g_vec}  m/s²")

    # 机器人连接 / connect arm
    arm = RobotArm()
    arm.connect()
    print("\n[连接 / connect] OK")

    # 使能电机 / enable motors
    arm.enable()
    print("[使能 / enable] OK")
    # arm.disable()  # 先失能测试

    # 切换到 MIT 模式（本示例 kp=2, kd=1；位置目标跟随当前 q）
    # / MIT mode with example gains kp=2, kd=1; pos tracks measured q
    arm.mode_mit(
        kp=np.full(arm.num_joints, 2.0),
        kd=np.full(arm.num_joints, 1.0),
    )
    print("[MIT模式 / MIT] OK（kp=2, kd=1）")

    # 启动控制循环，频率使用配置默认值 (500 Hz)
    # / Start control loop at YAML default rate (often 500 Hz)
    arm.start_control_loop(gravity_compensation_controller, rate=arm._rate)
    print(f"[控制循环 / control loop] 启动 @ {arm._rate} Hz / started @ {arm._rate} Hz")
    print("-" * 60)
    print(f"{'step':>4}  tau_g (N·m)")
    print("-" * 60)

    try:
        while _running:
            time.sleep(0.01)   # 主线程只负责保活，打印在回调中完成 / main thread idle; prints in callback
    finally:
        print("\n[停止 / stop] 关闭控制循环... / stopping control loop...")
        arm.disconnect()
        print("[完成 / done] 已安全断开连接 / safely disconnected")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""单电机控制测试（motorbridge SDK）/ Single-motor interactive test (motorbridge).

English: MIT / POS_VEL / VEL modes, enable/disable, zeroing, state.

用法 / Usage:
    python example/1_damiao_text.py

直接创建 ``Controller``，按 yaml 关节配置注册电机，
依次演示 MIT / POS_VEL / VEL，支持使能、回零、状态读取。
/ One ``Controller`` + motor from YAML joint profile; demos three modes.

交互命令 / Commands:
    mit <pos_deg> [<vel> <kp> <kd> <tau>]  — MIT
    posvel <pos_deg> [<vlim>]              — POS_VEL
    vel <vel_rad_s>                         — VEL
    enable                                  — 使能 / enable
    disable                                 — 去使能 / disable
    set_zero                                — 软件零位 / set zero
    mode <mit|posvel|vel>                   — 切换模式 / switch mode
    state                                   — 打印状态 / print state
    q / quit                                — 退出 / quit
"""

import sys
import signal
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from motorbridge import Controller, Mode
from reBotArm_control_py.actuator import is_dm_serial_channel

CHANNEL = "COM3"         
MOTOR_ID = 0x01
FEEDBACK_ID = 0x11
MODEL = "4340P"


def signal_handler(sig, frame):
    print("\n[ctrl+c] 退出 / exit")
    sys.exit(0)


def main() -> None:
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    print(f"连接到 {CHANNEL} ... / Connecting to {CHANNEL} ...")
    ch = CHANNEL.strip()
    if is_dm_serial_channel(ch):
        ctrl = Controller.from_dm_serial(ch, 921600)
    else:
        ctrl = Controller(ch)
    motor = ctrl.add_damiao_motor(MOTOR_ID, FEEDBACK_ID, MODEL)
    print(f"电机已注册 / motor registered: id={MOTOR_ID:#04x} feedback={FEEDBACK_ID:#04x} model={MODEL}")

    def do_enable() -> None:
        ctrl.enable_all()
        time.sleep(0.3)
        print("电机已使能 / enabled")

    def do_disable() -> None:
        ctrl.disable_all()
        print("电机已去使能 / disabled")

    def do_set_zero() -> None:
        st = motor.get_state()
        for _ in range(10):
            motor.request_feedback()
            st = motor.get_state()
            if st is not None and st.status_code == 0:
                break
            time.sleep(0.05)
        motor.set_zero_position()
        print("软件零位已设置 / software zero set")

    pv_pos_kp = 150.0
    pv_pos_ki = 0.5
    pv_vel_kp = 0.0125
    pv_vel_ki = 0.004

    def do_mode(args: list) -> None:
        nonlocal pv_pos_kp, pv_pos_ki, pv_vel_kp, pv_vel_ki
        if not args:
            print("用法 / usage: mode <mit|posvel|vel> [pos_kp] [pos_ki] [vel_kp] [vel_ki]")
            return
        m = args[0].lower()
        if m == "mit":
            motor.ensure_mode(Mode.MIT, 1000)
            print("切换到 MIT 模式 / switched to MIT")
        elif m == "posvel":
            if len(args) >= 5:
                pv_pos_kp = float(args[1])
                pv_pos_ki = float(args[2])
                pv_vel_kp = float(args[3])
                pv_vel_ki = float(args[4])
            motor.write_register_f32(25, pv_vel_kp)  # KP_ASR / vel-loop Kp
            motor.write_register_f32(26, pv_vel_ki)  # KI_ASR / vel-loop Ki
            motor.write_register_f32(27, pv_pos_kp)  # KP_APR / pos-loop Kp
            motor.write_register_f32(28, pv_pos_ki)  # KI_APR / pos-loop Ki
            time.sleep(0.02)
            print(f"PID 参数已写入 / PID written: pos_kp={pv_pos_kp} pos_ki={pv_pos_ki} "
                  f"vel_kp={pv_vel_kp} vel_ki={pv_vel_ki}")
            motor.ensure_mode(Mode.POS_VEL, 1000)
            print("切换到 POS_VEL 模式 / switched to POS_VEL")
        elif m == "vel":
            motor.ensure_mode(Mode.VEL, 1000)
            print("切换到 VEL 模式 / switched to VEL")
        else:
            print(f"未知模式 / unknown mode: {m}，可用 / valid: mit / posvel / vel")

    def do_state() -> None:
        st = None
        for _ in range(10):
            motor.request_feedback()         # 再请求反馈 / request feedback
            ctrl.poll_feedback_once()        # 轮询处理 / poll bus
            time.sleep(0.005)                # 等响应（~CAN 1ms）/ wait for reply
            st = motor.get_state()          # 读状态 / read state
            if st is not None and st.status_code == 0:
                break
        if st is None:
            print("无反馈数据 / no feedback")
            return
        print(f"pos={st.pos*180/3.14159:+.4f}deg  "
              f"vel={st.vel*180/3.14159:+.4f}deg/s  "
              f"torq={st.torq:+.4f}  "
              f"status={st.status_code}")

    def do_mit(args: list) -> None:
        if not args:
            print("用法 / usage: mit <pos_deg> [<vel> <kp> <kd> <tau>]")
            return
        pos = float(args[0]) * 3.14159265358979 / 180.0
        vel = float(args[1]) if len(args) > 1 else 0.0
        kp = float(args[2]) if len(args) > 2 else 10.0
        kd = float(args[3]) if len(args) > 3 else 2.0
        tau = float(args[4]) if len(args) > 4 else 0.0
        motor.send_mit(pos, vel, kp, kd, tau)

    def do_posvel(args: list) -> None:
        nonlocal pv_pos_kp, pv_pos_ki, pv_vel_kp, pv_vel_ki
        if not args:
            print("用法 / usage: posvel <pos_deg> [<vlim>] 或 / or posvel <pos_deg> <vlim> <pos_kp> <pos_ki> <vel_kp> <vel_ki>")
            return
        pos = float(args[0]) * 3.14159265358979 / 180.0
        vlim = float(args[1]) if len(args) > 1 else 2.0
        if len(args) >= 6:
            pv_pos_kp = float(args[2])
            pv_pos_ki = float(args[3])
            pv_vel_kp = float(args[4])
            pv_vel_ki = float(args[5])
            motor.write_register_f32(25, pv_vel_kp)  # KP_ASR / vel-loop Kp
            motor.write_register_f32(26, pv_vel_ki)  # KI_ASR / vel-loop Ki
            motor.write_register_f32(27, pv_pos_kp)  # KP_APR / pos-loop Kp
            motor.write_register_f32(28, pv_pos_ki)  # KI_APR / pos-loop Ki
            print(f"PID 参数已更新 / PID updated: pos_kp={pv_pos_kp} pos_ki={pv_pos_ki} "
                  f"vel_kp={pv_vel_kp} vel_ki={pv_vel_ki}")
            time.sleep(0.02)
        motor.send_pos_vel(pos, vlim)

    def do_vel(args: list) -> None:
        if not args:
            print("用法 / usage: vel <vel_rad_s>")
            return
        vel = float(args[0])
        motor.send_vel(vel)

    COMMANDS = {
        "enable": (do_enable, []),
        "disable": (do_disable, []),
        "set_zero": (do_set_zero, []),
        "state": (do_state, []),
        "mode": (do_mode, ""),
        "mit": (do_mit, "<pos_deg> [<vel> <kp> <kd> <tau>]"),
        "posvel": (do_posvel, "<pos_deg> [<vlim>]"),
        "vel": (do_vel, "<vel_rad_s>"),
    }

    print("\n命令 / commands: enable / disable / set_zero / mode / mit / posvel / vel / state / q")
    print("提示 / tip: mode 在下一条运动指令前生效 / mode applies before next motion cmd\n")

    try:
        while True:
            try:
                line = input("> ").strip()
            except EOFError:
                break
            if not line:
                continue

            parts = line.split()
            cmd = parts[0].lower()
            args = parts[1:]

            if cmd in ("q", "quit", "exit"):
                print("退出 / bye")
                break

            if cmd not in COMMANDS:
                print(f"未知命令 / unknown: {cmd}，可用 / available: {' / '.join(COMMANDS)}")
                continue

            fn, help_hint = COMMANDS[cmd]
            if help_hint and not args and fn in (do_mode, do_mit, do_posvel, do_vel):
                print(f"用法 / usage: {cmd} {help_hint}")
                continue

            try:
                if help_hint and fn not in (do_mode, do_mit, do_posvel, do_vel):
                    fn()
                elif fn == do_mode:
                    fn(args)
                elif fn == do_mit:
                    fn(args)
                elif fn == do_posvel:
                    fn(args)
                elif fn == do_vel:
                    fn(args)
                else:
                    fn()
            except Exception as e:
                print(f"错误 / error: {e}")

    finally:
        ctrl.disable_all()
        ctrl.shutdown()
        ctrl.close()


if __name__ == "__main__":
    main()

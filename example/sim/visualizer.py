#!/usr/bin/env python3
"""MeshCat 可视化封装 — URDF 场景 / MeshCat URDF visualizer helper.

用法 / Usage:
    from example.sim.visualizer import Visualizer
    viz = Visualizer()
    viz.update(q)  # q: 关节角度 (nq,) / joint vector (nq,)

功能 / Features:
    - 绘制 3D 折线路径（参考 / 实际）/ 3D polyline ref vs actual path
    - 播放关节轨迹动画（逐帧 + 路径同步）/ joint-space animation with path sync
    - 显示 IK 目标位姿（三色轴 + 球体标记）/ IK target triad + marker sphere
"""

import sys
import time
from pathlib import Path

import meshcat
import meshcat.geometry as mcg
import numpy as np
import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from reBotArm_control_py.kinematics import _get_default_urdf_path


class Visualizer:
    """MeshCat + Pinocchio 机器人可视化器 / Robot visualizer.

    支持 / Supports:
        - 逐帧更新机械臂姿态 / per-frame configuration updates
        - 绘制末端参考路径（灰色）和已走路径（绿色）/ planned (gray) vs visited (green) EE path
        - IK 目标位姿可视化（三色轴 + 球体）/ IK target triad + sphere
        - 轨迹动画播放 / trajectory playback helper
    """

    def __init__(self, open_browser: bool = True):
        urdf_path = _get_default_urdf_path()
        pkg_dir = str(Path(urdf_path).parents[2])

        self._model = pin.buildModelFromUrdf(urdf_path)
        self._data = self._model.createData()

        self._visual_model = pin.buildGeomFromUrdf(
            self._model, urdf_path, pin.GeometryType.VISUAL, package_dirs=[pkg_dir]
        )
        self._visual_data = self._visual_model.createData()

        # zmq_url=None → 新服务器；字符串则连已有实例 / new server vs connect to existing zmq_url
        self._meshcat_viz = meshcat.Visualizer(zmq_url=None)

        self._viz = MeshcatVisualizer(
            self._model,
            collision_model=None,
            visual_model=self._visual_model,
            data=self._data,
            visual_data=self._visual_data,
        )
        self._viz.initViewer(self._meshcat_viz, loadModel=False)
        self._viz.loadViewerModel()

        if open_browser:
            print(f"MeshCat 地址 / URL: {self._meshcat_viz.url()}")

    @property
    def meshcat(self):
        """暴露底层 meshcat.Visualizer / Underlying meshcat ``Visualizer``."""
        return self._meshcat_viz

    def update(self, q) -> None:
        """更新机器人显示位姿 / Update displayed configuration. ``q`` may be list or ndarray."""
        q = np.asarray(q)
        if q.shape != (self._model.nq,):
            raise ValueError(f"q 须为形状 ({self._model.nq},)，实际为 {q.shape} / q must be ({self._model.nq},), got {q.shape}")
        self._viz.display(q)

    def neutral(self) -> None:
        """恢复到中位配置 / Neutral configuration."""
        q0 = pin.neutral(self._model)
        self._viz.display(q0)

    @property
    def nq(self) -> int:
        return self._model.nq

    @property
    def model(self):
        """暴露 model / Expose ``pin.Model`` (e.g. for ``compute_fk``)."""
        return self._model

    # ── 路径绘制 / Path drawing ─────────────────────────────────────────────────

    def draw_path(
        self,
        points_xyz: list,
        node_name: str,
        color: int = 0x00aaff,
    ) -> None:
        """在场景中绘制 3D 折线路径 / Draw 3D polyline.

        Args:
            points_xyz: 三维点列表 [[x,y,z], ...] / list of XYZ waypoints
            node_name:  MeshCat 节点名 / MeshCat node name
            color:      RGB 十六进制 / hex RGB (default light blue)
        """
        if len(points_xyz) < 2:
            return
        pts = np.array(points_xyz, dtype=np.float32).T
        line = mcg.Line(
            mcg.PointsGeometry(pts),
            mcg.LineBasicMaterial(color=color, linewidth=2),
        )
        self._meshcat_viz[node_name].set_object(line)

    def draw_ref_path(self, points_xyz: list) -> None:
        """绘制灰色参考路径（笛卡尔规划）/ Gray reference Cartesian path."""
        self.draw_path(points_xyz, "traj_path/ref", color=0x888888)

    def draw_actual_path(self, points_xyz: list, color: int = 0x00cc44) -> None:
        """绘制已走路径（绿色）/ Visited path (green)."""
        self.draw_path(points_xyz, "traj_path/actual", color=color)

    def clear_paths(self) -> None:
        """清除所有轨迹路径节点 / Clear traj path nodes."""
        for name in ("traj_path/ref", "traj_path/actual"):
            try:
                del self._meshcat_viz[name]
            except Exception:
                pass

    # ── IK 目标可视化 / IK target viz ─────────────────────────────────────────

    def show_ik_pose(
        self,
        xyz: np.ndarray,
        R: np.ndarray,
        q: np.ndarray,
    ) -> None:
        """显示 IK 求解结果（目标位姿 + 关节角）/ Show IK solution pose + q.

        可视化 / Renders:
            - 目标位姿：RGB 轴 + 红球 / target triad + red sphere
            - 机械臂：更新到求解 q / robot at solved ``q``

        Args:
            xyz: 目标位置 / target position [x, y, z]
            R:   3×3 旋转矩阵 / rotation matrix
            q:   关节角 (nq,) / joint vector
        """
        # 构建 4x4 齐次变换 / build 4x4 homogeneous transform
        H = np.eye(4)
        H[:3, :3] = R
        H[:3, 3] = xyz

        # 显示目标坐标系（三色轴）/ target frame triad
        self._meshcat_viz["target/frame"].set_object(mcg.triad())
        self._meshcat_viz["target/frame"].set_transform(H)

        # 目标位置标记（红球）/ target position sphere
        self._meshcat_viz["target/ball"].set_object(
            mcg.Sphere(0.015),
            mcg.MeshLambertMaterial(color=0xFF3300),
        )
        self._meshcat_viz["target/ball"].set_transform(H)

        # 更新机械臂 / update robot mesh
        self.update(np.asarray(q))

    def clear_ik_pose(self) -> None:
        """清除 IK 目标可视化 / Remove IK target nodes."""
        for name in ("target/frame", "target/ball"):
            try:
                del self._meshcat_viz[name]
            except Exception:
                pass

    # ── 轨迹线 / EE trajectory line ───────────────────────────────────────────

    def plot_trajectory_line(
        self,
        joint_traj: list,
        color: int = 0xFF3300,
        name: str = "ee_trajectory",
    ) -> None:
        """在 MeshCat 中绘制末端轨迹线 / Draw EE polyline in MeshCat.

        Args:
            joint_traj: 关节角序列 / sequence of ``(nq,)`` arrays or trajectory points
            color:      RGB / RGB int
            name:       MeshCat 节点名 / node name
        """
        from reBotArm_control_py.kinematics import compute_fk

        positions = []
        for pt in joint_traj:
            q = np.asarray(pt.q) if hasattr(pt, "q") else np.asarray(pt)
            _, _, T = compute_fk(self._model, q)
            positions.append(T[:3, 3])
        positions = np.array(positions, dtype=float)

        if len(positions) < 2:
            return
        self.clear_trajectory_line(name)
        self._meshcat_viz[name].set_object(
            mcg.Line(
                mcg.PointsGeometry(positions),
                mcg.LineBasicMaterial(color=color, linewidth=2),
            )
        )

    def clear_trajectory_line(self, name: str = "ee_trajectory") -> None:
        """清除 MeshCat 轨迹线 / Delete trajectory line node."""
        try:
            del self._meshcat_viz[name]
        except Exception:
            pass

    # ── 轨迹播放 / Trajectory playback ─────────────────────────────────────────

    def play_trajectory(
        self,
        name: str,
        dt: float,
        q_list: list,
        path: list | None = None,
    ) -> None:
        """播放关节轨迹动画 / Animate joint trajectory.

        流程 / Steps:
            1. 参考路径（灰）/ draw gray ref
            2. 逐帧 ``update``（dt）/ step robot at dt
            3. 同步已走路径（绿）/ append visited path

        Args:
            name: 日志用名称 / label for logs
            dt:   帧间隔 [s] / frame period
            q_list: 关节序列 / list of q
            path: 末端 XYZ 序列（可选）/ optional EE positions [[x,y,z],...]
        """
        print(
            f"[viz] 播放轨迹 / play: {name}  点数 / n={len(q_list)}  dt={dt:.3f}s",
            flush=True,
        )

        if path:
            self.draw_ref_path(path)

        visited = []
        for i, q in enumerate(q_list):
            self.update(np.asarray(q))
            if path and i < len(path):
                visited.append(path[i])
                self.draw_actual_path(visited)
            time.sleep(dt)

        print(f"[viz] 轨迹 '{name}' 完毕 / done", flush=True)
        time.sleep(1.0)

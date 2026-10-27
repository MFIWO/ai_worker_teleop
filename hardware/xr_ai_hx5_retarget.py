# Adapted from xr_ai_teleop/teleop/robot_control/robot_hand_hx5_d20.py.
# Apache-2.0; see xr_ai_assets/LICENSE.xr_ai_teleop.
"""Apple Vision Pro hand-landmark retargeting for ROBOTIS HX5-D20-MLT.

The input layout is the 25 x 3 landmark array emitted by TeleVuer.  The output
is the native 20-joint layout used by the official robotis_hand URDF and the
AI Worker SH5 ROS 2 controllers.
"""

import logging
from pathlib import Path
import sys
import threading
import time

import numpy as np
from hardware.thumb_flexion import thumb_flexion


HX5_D20_NUM_JOINTS = 20
LEFT_JOINT_NAMES = tuple(f"finger_l_joint{i}" for i in range(1, 21))
RIGHT_JOINT_NAMES = tuple(f"finger_r_joint{i}" for i in range(1, 21))

# Official HX5-D20 rev1 limits. Joint blocks are thumb, index, middle, ring,
# little; every block contains abduction followed by three flexion joints.
LEFT_LOWER = np.array([-1.57, 0.0, -1.57, -1.57] + [-0.6, 0.0, 0.0, 0.0] * 4)
LEFT_UPPER = np.array([1.57, 3.14, 0.0, 0.0] + [0.6, 2.0, 1.57, 1.57] * 4)
RIGHT_LOWER = np.array([-1.57, -3.14, 0.0, 0.0] + [-0.6, 0.0, 0.0, 0.0] * 4)
RIGHT_UPPER = np.array([1.57, 0.0, 1.57, 1.57] + [0.6, 2.0, 1.57, 1.57] * 4)

_REPO_ROOT = Path(__file__).resolve().parents[2]
_ASSETS_ROOT = Path(__file__).with_name("xr_ai_assets")
_HX5_D20_CONFIG_PATH = _ASSETS_ROOT / "HX5_D20.yml"
_HX5_D20_POSITION_AXES = np.array(
    [
        [-1.0, 0.0, 0.0],
        [0.0, 0.0, -1.0],
        [0.0, -1.0, 0.0],
    ],
    dtype=np.float64,
)


def resolve_hx5_d20_urdf(side, ai_worker_urdf=None):
    """Resolve a hand URDF from the selected AI Worker checkout."""
    if side not in ("left", "right"):
        raise ValueError("side must be 'left' or 'right'")

    relative_path = Path("common") / "hx5_d20" / f"hx5_d20_{side}.urdf"
    if ai_worker_urdf is not None:
        arm_urdf = Path(ai_worker_urdf).expanduser().resolve()
        for parent in arm_urdf.parents:
            if parent.name == "urdf":
                return (parent / relative_path).resolve()
        raise ValueError(
            "AI Worker arm URDF must be located below an ffw_description/urdf "
            f"directory, got: {arm_urdf}"
        )

    workspace_root = Path(__file__).resolve().parents[3]
    candidates = (
        workspace_root
        / "external_repos"
        / "ai_worker"
        / "ffw_description"
        / "urdf"
        / relative_path,
        workspace_root
        / "ai_worker"
        / "ffw_description"
        / "urdf"
        / relative_path,
    )
    for candidate in candidates:
        if candidate.is_file():
            return candidate.resolve()
    # Prefer the repository layout used by this workspace in the eventual
    # FileNotFoundError so the diagnostic points at the expected location.
    return candidates[0].resolve()


def _unit(vector):
    norm = float(np.linalg.norm(vector))
    if norm < 1e-8:
        return np.zeros(3)
    return vector / norm


def _bend_angle(a, b, c):
    """Return zero for a straight chain and positive radians for flexion."""
    first = _unit(a - b)
    second = _unit(c - b)
    if not np.any(first) or not np.any(second):
        return 0.0
    interior = np.arccos(np.clip(np.dot(first, second), -1.0, 1.0))
    return float(np.pi - interior)


class HX5D20Retargeter:
    """Geometric 25-landmark to 20-joint retargeter with EMA smoothing."""

    # TeleVuer/OpenXR chains: metacarpal, MCP, PIP, DIP, fingertip.
    FINGER_CHAINS = (
        (5, 6, 7, 8, 9),
        (10, 11, 12, 13, 14),
        (15, 16, 17, 18, 19),
        (20, 21, 22, 23, 24),
    )

    def __init__(
        self,
        side,
        smoothing_alpha=0.35,
        thumb_yaw_gain=1.0,
        thumb_yaw_max=1.2,
        thumb_pitch_max=0.7,
    ):
        if side not in ("left", "right"):
            raise ValueError("side must be 'left' or 'right'")
        self.side = side
        self.alpha = float(np.clip(smoothing_alpha, 0.0, 1.0))
        self.lower = LEFT_LOWER if side == "left" else RIGHT_LOWER
        self.upper = LEFT_UPPER if side == "left" else RIGHT_UPPER
        self.thumb_yaw_gain = max(0.0, float(thumb_yaw_gain))
        self.thumb_yaw_max = max(0.0, float(thumb_yaw_max))
        self.thumb_pitch_max = max(0.0, float(thumb_pitch_max))
        self._thumb_yaw_neutral = None
        self._filtered = None

    def retarget(self, landmarks):
        points = np.asarray(landmarks, dtype=np.float64).reshape(25, 3)
        if not np.all(np.isfinite(points)) or np.count_nonzero(np.linalg.norm(points, axis=1) > 1e-7) < 20:
            raise ValueError("HX5-D20 retargeting requires a valid 25-point hand skeleton.")

        wrist = points[0]
        palm_forward = _unit(points[10] - wrist)
        palm_side = _unit(points[5] - points[20])  # little -> index, anatomical for either hand
        if not np.any(palm_forward) or not np.any(palm_side):
            raise ValueError("Degenerate palm landmarks.")

        command = np.zeros(20, dtype=np.float64)

        # HX5 joint1 is the root pitch/opposition axis and joint2 is the
        # in-palm yaw axis.  The two hands use mirrored command signs.
        thumb = (1, 2, 3, 4)
        thumb_yaw = _bend_angle(wrist, points[1], points[2])
        # The official SH5 open-hand pose is zero for both root joints.  Remove
        # the human thumb's non-zero resting yaw instead of sending it directly.
        if self._thumb_yaw_neutral is None:
            self._thumb_yaw_neutral = thumb_yaw
        thumb_yaw = np.clip(
            (thumb_yaw - self._thumb_yaw_neutral) * self.thumb_yaw_gain,
            0.0,
            self.thumb_yaw_max,
        )

        # Drive root pitch only from an actual thumb-index pinch.  This avoids
        # the old behavior where an in-plane thumb angle pushed joint1 almost
        # 90 degrees backwards.  Ratios make the thresholds hand-size agnostic:
        # pitch begins below 0.75 palm lengths and reaches full travel at 0.15.
        palm_length = max(float(np.linalg.norm(points[10] - wrist)), 1e-6)
        pinch_ratio = float(np.linalg.norm(points[4] - points[9])) / palm_length
        pinch_amount = float(np.clip((0.75 - pinch_ratio) / 0.60, 0.0, 1.0))
        thumb_pitch = self.thumb_pitch_max * pinch_amount
        thumb_pip = _bend_angle(points[1], points[2], points[3])
        thumb_dip = _bend_angle(points[2], points[3], points[4])
        if self.side == "left":
            command[:4] = (-thumb_pitch, thumb_yaw, -thumb_pip, -thumb_dip)
            abduction_sign = 1.0
        else:
            command[:4] = (thumb_pitch, -thumb_yaw, thumb_pip, thumb_dip)
            abduction_sign = -1.0

        for finger_index, (metacarpal, mcp, pip, dip, tip) in enumerate(self.FINGER_CHAINS):
            finger_direction = _unit(points[pip] - points[mcp])
            abduction = abduction_sign * np.arctan2(
                np.dot(finger_direction, palm_side),
                np.dot(finger_direction, palm_forward),
            )
            base = 4 + finger_index * 4
            command[base : base + 4] = (
                abduction,
                _bend_angle(points[metacarpal], points[mcp], points[pip]),
                _bend_angle(points[mcp], points[pip], points[dip]),
                _bend_angle(points[pip], points[dip], points[tip]),
            )

        command = np.clip(command, self.lower, self.upper)
        if self._filtered is None:
            self._filtered = command
        else:
            self._filtered = self.alpha * command + (1.0 - self.alpha) * self._filtered
        return self._filtered.copy()


class HX5D20DexPilotRetargeter:
    """HX5 wrapper around the same dex_retargeting stack used by RH5DG2."""

    def __init__(
        self,
        side,
        smoothing_alpha=1.0,
        hand_scale=1.0,
        urdf_path=None,
        ai_worker_urdf=None,
    ):
        if side not in ("left", "right"):
            raise ValueError("side must be 'left' or 'right'")

        local_dex_src = Path(__file__).resolve().parents[1] / "vendor"
        if local_dex_src.exists() and str(local_dex_src) not in sys.path:
            sys.path.insert(0, str(local_dex_src))
        try:
            import yaml
            try:
                from dex_retargeting import RetargetingConfig
            except ImportError:
                # Older upstream releases do not re-export this class.
                from dex_retargeting.retargeting_config import RetargetingConfig
        except ImportError as exc:
            raise RuntimeError(
                "HX5-D20 DexPilot mode requires xr_tele's dex-retargeting package. "
                "From xr_tele run: git submodule update --init teleop/robot_control/dex-retargeting "
                "and from xr_tele/teleop run: python -m pip install -e robot_control/dex-retargeting"
            ) from exc

        self.side = side
        self.lower = (LEFT_LOWER if side == "left" else RIGHT_LOWER).copy()
        self.upper = (LEFT_UPPER if side == "left" else RIGHT_UPPER).copy()

        with _HX5_D20_CONFIG_PATH.open("r", encoding="utf-8") as config_file:
            config = yaml.safe_load(config_file)
        side_config = dict(config[side])
        if urdf_path is None:
            urdf_path = resolve_hx5_d20_urdf(side, ai_worker_urdf)
        self.urdf_path = Path(urdf_path).expanduser().resolve()
        if not self.urdf_path.is_file():
            raise FileNotFoundError(f"HX5-D20 {side} URDF not found: {self.urdf_path}")

        # Keep MCP/IP flexion observable even when thumb-tip position can be
        # matched by moving only the root axes. Optimize the other 18 joints
        # with the tracked thumb bends fixed in the SAME kinematic solve.
        target_joint_names = list(side_config["target_joint_names"])
        self.thumb_joint_names = target_joint_names[2:4]
        side_config["target_joint_names"] = [n for n in target_joint_names if n not in self.thumb_joint_names]
        side_config["urdf_path"] = str(self.urdf_path)
        side_config["low_pass_alpha"] = float(np.clip(smoothing_alpha, 0.0, 1.0))
        side_config["scaling_factor"] = float(side_config.get("scaling_factor", 1.0)) * float(hand_scale)
        RetargetingConfig.set_default_urdf_dir(str(_ASSETS_ROOT))
        self.retargeting = RetargetingConfig.from_dict(side_config).build()
        self.retargeting.optimizer.opt.set_maxeval(14)
        self.retargeting.optimizer.opt.set_maxtime(.006)
        if set(self.retargeting.optimizer.fixed_joint_names) != set(self.thumb_joint_names):
            raise RuntimeError('Unexpected HX5 fixed-joint mapping')
        self.indices = np.asarray(self.retargeting.optimizer.target_link_human_indices, dtype=np.int64)
        if self.indices.ndim != 2 or self.indices.shape[0] != 2:
            raise RuntimeError(f"HX5-D20 DexPilot expected 2xN human indices, got {self.indices.shape}")

        self.retargeting_to_hardware = np.asarray(
            [self.retargeting.joint_names.index(name) for name in target_joint_names],
            dtype=np.int64,
        )
        if len(self.retargeting_to_hardware) != HX5_D20_NUM_JOINTS:
            raise RuntimeError("HX5-D20 DexPilot configuration must target exactly 20 joints.")

        # HX5 is open at q=0; unlike RH5DG2, several mirrored thumb joints have
        # negative lower limits, so lower-limit initialization is not an open hand.
        robot_qpos = np.zeros(self.retargeting.optimizer.robot.dof, dtype=np.float32)
        self.retargeting.set_qpos(robot_qpos)
        if self.retargeting.filter is not None:
            self.retargeting.filter.reset()

    def retarget(self, landmarks):
        points = np.asarray(landmarks, dtype=np.float64).reshape(25, 3)
        if not np.all(np.isfinite(points)) or np.count_nonzero(np.linalg.norm(points, axis=1) > 1e-7) < 20:
            raise ValueError("HX5-D20 retargeting requires a valid 25-point hand skeleton.")
        source = points[self.indices[0, :]]
        target = points[self.indices[1, :]]
        input_vectors = target - source
        hx5_vectors = input_vectors @ _HX5_D20_POSITION_AXES.T
        bends = dict(zip(self.thumb_joint_names, thumb_flexion(points, self.side)))
        fixed = np.array([bends[n] for n in self.retargeting.optimizer.fixed_joint_names])
        q = self.retargeting.retarget(hx5_vectors, fixed_qpos=fixed)[self.retargeting_to_hardware]
        return np.clip(np.asarray(q, dtype=np.float64), self.lower, self.upper)

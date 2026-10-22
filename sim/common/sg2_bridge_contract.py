"""[ADDED] Pure-Python SG2 DDS trajectory contract (no Isaac/ROS imports).

The real VR controller publishes two arm JointTrajectory topics.  The final
trajectory may also include gripper_[lr]_joint1, forwarded from Quest trigger.
Reject unintended robot/base/other-arm commands rather than silently moving
an unrelated articulation joint.
"""

import math


ARM_JOINTS = {
    side: tuple(f"arm_{side}_joint{number}" for number in range(1, 8))
    for side in ("l", "r")
}
GRIPPER_MASTER = {side: f"gripper_{side}_joint1" for side in ("l", "r")}
ARM_TOPIC = {
    "r": "/leader/joint_trajectory_command_broadcaster_right/joint_trajectory",
    "l": "/leader/joint_trajectory_command_broadcaster_left/joint_trajectory",
}
JOINT_STATES_TOPIC = "/joint_states"
TF_TOPIC = "/tf"
READY_ELBOW_RAD = -1.57


def allowed_names(side, include_gripper=True):
    if side not in ARM_JOINTS:
        raise ValueError(f"Unknown side: {side!r}")
    return set(ARM_JOINTS[side]) | ({GRIPPER_MASTER[side]} if include_gripper else set())


def parse_arm_trajectory(side, names, positions, available, include_gripper=True):
    """Validate all-or-nothing input, returning {joint: finite_position}.

    Return None for an invalid/empty packet.  The caller may log its rejection;
    this intentionally does not alter target/activation state.
    """
    names = list(names)
    positions = list(positions)
    expected = allowed_names(side, include_gripper)
    if not names or len(names) != len(positions) or len(names) != len(set(names)):
        return None
    if not set(names).issubset(expected) or not set(names).issubset(set(available)):
        return None
    try:
        numeric = [float(value) for value in positions]
    except (ValueError, TypeError):
        return None
    if not all(math.isfinite(v) for v in numeric):
        return None
    return dict(zip(names, numeric))


def bounded_target(current, requested, lower, upper, max_step):
    """Reject nonfinite/out-of-range commands; rate-limit accepted targets."""
    values = (current, requested, lower, upper, max_step)
    if not all(math.isfinite(float(value)) for value in values):
        return None
    if lower > upper or max_step <= 0 or requested < lower or requested > upper:
        return None
    return max(lower, min(upper, current + max(-max_step, min(max_step, requested - current))))

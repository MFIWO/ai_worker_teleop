"""ROBOTIS mode 1 with independently tracked thumb MCP/IP and bounded solve."""
import numpy as np

from hardware.thumb_flexion import apply_thumb_flexion


def retarget_robotis(retargeter, points, side):
    # Validate before changing solver/filter state.
    apply_thumb_flexion(np.zeros(20), points, side)
    result = retargeter.retarget(points)
    goal = apply_thumb_flexion(result.robot_qpos, points, side)
    retargeter.last_qpos = goal[retargeter.optimizer.idx_pin2target].astype(np.float32)
    retargeter.filter.y = goal.copy()
    return goal


def main():
    import rclpy
    from scripts.teleop_retargeting import RetargetingTeleop

    class ThumbRetargetingTeleop(RetargetingTeleop):
        def __init__(self):
            super().__init__()
            for retargeter in (self.left_retargeter, self.right_retargeter):
                retargeter.filter.alpha = 1.
                retargeter.optimizer.opt.set_maxeval(14)
                retargeter.optimizer.opt.set_maxtime(.006)
            self.get_logger().info('[HAND MODE 1] ROBOTIS + tracked thumb MCP/IP; no output low-pass; 14 evaluations / 6ms solve budget')

        def run_hand(self, side, msg):
            if len(msg.joints) != 21:
                return
            points = self.convert_msg_to_numpy(msg)
            if not np.isfinite(points).all():
                return
            retargeter = getattr(self, side+'_retargeter')
            try:
                goal = retarget_robotis(retargeter, points, side)
            except ValueError as error:
                self.get_logger().warning(str(error), throttle_duration_sec=2.)
                return
            getattr(self, 'publish_trajectory_'+side)(goal)

        def run_teleop_left(self, msg):
            self.run_hand('left', msg)

        def run_teleop_right(self, msg):
            self.run_hand('right', msg)

    rclpy.init()
    node = ThumbRetargetingTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

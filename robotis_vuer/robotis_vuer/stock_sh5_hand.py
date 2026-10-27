"""Use ROBOTIS SH5 landmark conversion inside the existing arm session.

The outer session supplies start/HOLD. Wrist/arm publishing is deliberately
not delegated here. Each HAND_MOVE emits only the newly received side, with
the original wrist-local landmark convention. BODY_MOVE never replays hands.
"""
import numpy as np
from scipy.spatial.transform import Rotation as R
from robotis_interfaces.msg import HandJoints

from robotis_vuer.vr_publisher_sh5 import VRTrajectoryPublisher as OriginalSH5


class StockSH5HandInput:
    on_hand_move=OriginalSH5.on_hand_move
    _extract_gesture_state=OriginalSH5._extract_gesture_state
    get_body_joint_matrix_from_flat=OriginalSH5.get_body_joint_matrix_from_flat
    process_hand_joints=OriginalSH5.process_hand_joints
    _set_vr_publishing_enabled=OriginalSH5._set_vr_publishing_enabled
    BODY_HEAD_TO_ROS_POSITION=OriginalSH5.BODY_HEAD_TO_ROS_POSITION

    def __init__(self,node):
        self.node=node
        self.required_vr_frames=[0,1,2,3,4,6,7,8,9,11,12,13,14,16,17,18,19,21,22,23,24]
        self.vr_hand_to_urdf=np.array([[0,-1,0],[-1,0,0],[0,0,-1]])
        self.vr_to_ros_matrix=np.array([[0.,0.,-1.],[-1.,0.,0.],[0.,1.,0.]])
        self.body_head_to_ros_rot=R.from_matrix(self.BODY_HEAD_TO_ROS_POSITION)
        self.low_pass_filter_alpha=1.
        self.vr_publishing_enabled=False
        self.current_odom=None
        self.left_hand_data=self.right_hand_data=None
        self.prev_poses_left=np.zeros((21,3));self.prev_poses_right=np.zeros((21,3))
        self.start_poses_left=self.start_poses_right=False
        self.head_transform_matrix=np.eye(4);self.head_inverse_matrix=np.eye(4)
        if not node.has_parameter('hand_pose_is_head_relative'):
            node.declare_parameter('hand_pose_is_head_relative',True)
        # Finger output is wrist-local; the head transform cancels exactly.
        # The unused wrist/base output remains suppressed by this adapter.
        self.hand_pose_is_head_relative=False
        self.left_hand_pos_pub=node.create_publisher(HandJoints,'/left_hand/hand_joint_pos',node.vr_stream_qos)
        self.right_hand_pos_pub=node.create_publisher(HandJoints,'/right_hand/hand_joint_pos',node.vr_stream_qos)
        # Required only by the original method's unused wrist-pose output.
        self.left_wrist_rviz_pub=self.right_wrist_rviz_pub=None
        self.wrist_offsets={s:dict(x=0.,y=0.,z=0.) for s in ('left','right')}
        self.wrist_vr_scale=1.4

    def get_clock(self):return self.node.get_clock()
    def get_logger(self):return self.node.get_logger()
    def _update_gesture_toggle(self,*_):pass  # outer session owns start/HOLD
    def publish_relative_pose(self,*_,**__):pass  # outer arm teleop is unchanged
    def _publish_zero_cmd_vel(self):pass  # outer session owns base HOLD

    def publish_event(self,event,enabled):
        self._set_vr_publishing_enabled(enabled,reset_references=True)
        if not enabled or not isinstance(event.value,dict):return
        stamp=self.get_clock().now().to_msg()
        for side in ('left','right'):
            if side in event.value:
                data=getattr(self,side+'_hand_data')
                raw=np.asarray(event.value[side],dtype=float)
                if raw.size==400 and np.isfinite(raw).all() and data is not None:
                    self.process_hand_joints(data,side,pose_stamp=stamp)

    def process_body(self,event,enabled):
        self._set_vr_publishing_enabled(enabled,reset_references=True)
        if not enabled:return  # session HOLD keeps posture; no zero-position motion
        try:
            value=getattr(event,'value',None)
            if not isinstance(value,dict):return
            body=value.get('body')
            if not isinstance(body,(list,tuple,np.ndarray)):return
            head=self.get_body_joint_matrix_from_flat(body,'head')
            if head is None:return
            self.head_transform_matrix=head
            self.head_inverse_matrix=np.linalg.inv(head)
        except Exception as error:
            self.get_logger().error('Original SH5 hand processing: '+str(error))

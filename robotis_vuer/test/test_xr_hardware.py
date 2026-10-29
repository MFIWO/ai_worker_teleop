"""[ADDED] Real rclpy with numerical fixtures on TEST domain232, no simulator/motors."""
from copy import deepcopy
import asyncio
import json
import os
import time
import unittest
from unittest.mock import patch

import numpy as np
import rclpy
from std_msgs.msg import Bool, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from scipy.spatial.transform import Rotation
from types import SimpleNamespace
from geometry_msgs.msg import TransformStamped, PoseStamped
from tf2_msgs.msg import TFMessage
from std_srvs.srv import Trigger
from robotis_vuer.vr_publisher_sg2_xr_resume import XRResumePublisher
from hardware.aux_controls import read_aux_limits
from hardware.xr_contract import GOAL_SPEED, ANGULAR_SPEED

from robotis_vuer.vr_publisher_ai_worker_hardware import HardwareXRPublisher
from robotis_vuer.vr_publisher_sg2_xr import XRDeltaPublisher
from robotis_vuer.xr_path_workspace import AnchorRayHint
from test_xr_resume import fixture
from xr_resume_smoke import signature


def aux_fixture():
    limits = {'head_joint1':(-.2317,.6951,4.8), 'head_joint2':(-.35,.35,4.8),
              'lift_joint':(-.5,0.,4.8), 'gripper_l_joint1':(0.,1.1,6.5), 'gripper_r_joint1':(0.,1.1,6.5)}
    return read_aux_limits('<robot>' + ''.join(
        f'<joint name="{k}"><limit lower="{lo}" upper="{hi}" velocity="{vel}"/></joint>'
        for k,(lo,hi,vel) in limits.items()) + '</robot>')


class HardwarePublisherTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if os.environ.get('ROS_DOMAIN_ID') != '232':
            raise unittest.SkipTest('test-domain232 only')
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.env=patch.dict(os.environ,{'AI_WORKER_XR_EXECUTION':'validation',
                                     'AI_WORKER_XR_MODEL':'sg2','AI_WORKER_XR_ALLOW_ARM':'0'})
        self.env.start()
        self.node=HardwareXRPublisher(start_server=False)

    def tearDown(self):
        self.node._set_arm_control_enabled(False,'test cleanup')
        self.node.destroy_node()
        self.env.stop()

    def capture(self):
        n=self.node
        h,inputs,robots,links=fixture()
        n._graph_check=lambda: None
        n._arm_tracking_is_fresh=lambda now=None: True
        n._require_still_robot=lambda now: None
        n._snapshot=lambda: deepcopy(links)
        now=time.monotonic(); ros=n.get_clock().now().nanoseconds*1e-9
        n._stopped_at,n._stopped_ros=now-1,ros-1
        n._robot_poses={s:(p.copy(),now-.01,ros-.01) for s,p in robots.items()}
        n.head_transform_matrix=h
        for s,p in inputs.items(): setattr(n,s+'_controller_matrix',p.copy())
        n.hardware_allow_arm=True
        n._set_arm_control_enabled(True,'fixture X+A')
        self.assertEqual(n._phase,'SETTLING')
        # Synthetic live receipts for ACTIVE tests, including feedback-wait guards.
        n.last_body_pose_sec = n._joint_rx = now
        n.last_controller_pose_sec = {s: now for s in inputs}
        n._command_rx = {s: now for s in inputs}
        return inputs,robots,links

    def test_observe_only_cannot_enable(self):
        self.node._capture=lambda: self.fail('observe-only must never capture/enable')
        self.node._set_arm_control_enabled(True,'X+A')
        self.assertFalse(self.node.arm_control_enabled)
        self.assertIsNone(self.node._session)

    def test_near_boundary_command_feedback_keeps_pc_active(self):
        self.capture()
        n = self.node
        n._phase = 'ACTIVE'
        names = [f'arm_l_joint{i}' for i in range(1,8)]
        n._hardware_manifest = {'limits': {k:[-3.14,3.14] for k in names}}
        n._hardware_manifest['limits']['arm_l_joint2'] = [0.,3.14]
        msg = JointTrajectory(joint_names=names)
        msg.points = [JointTrajectoryPoint(positions=[0.,-1.5e-7,0.,-1.57,0.,0.,0.])]
        n._command_feedback(msg, 'left')
        self.assertTrue(n.arm_control_enabled)
        self.assertIn('left',n._command_rx)

    def test_hardware_capture_does_not_expand_elbow_or_shell(self):
        self.capture()
        self.assertEqual(self.node.goal_speed,GOAL_SPEED)
        self.assertEqual(self.node.angular_speed,ANGULAR_SPEED)
        for hint in self.node._session.templates.values():
            self.assertIsInstance(hint,AnchorRayHint)
            self.assertLessEqual(hint.maximum,.94*(hint.upper+hint.forearm)+1e-8)

    def test_hardware_real_mode_cannot_use_test_domain(self):
        with patch.dict(os.environ,{'AI_WORKER_XR_EXECUTION':'hardware','ROS_DOMAIN_ID':'232'}):
            with self.assertRaises(RuntimeError): self.node._validate_runtime_domain()

    def test_sim_class_still_refuses_robot_domain(self):
        with patch.dict(os.environ,{'ROS_DOMAIN_ID':'30'}):
            with self.assertRaises(RuntimeError): XRDeltaPublisher._validate_runtime_domain(self.node)

    def test_former_50mm_tracking_stop_keeps_following(self):
        _,_,links=self.capture()
        self.node._phase='ACTIVE'
        links['left']['wrist'][0,3]+=.06
        self.node._command_rx = {s: time.monotonic() for s in ('left', 'right')}
        count=self.node.published_pairs
        self.node._publish_synced_pose_frame_if_ready()
        self.assertTrue(self.node.arm_control_enabled)
        self.assertEqual(count+1,self.node.published_pairs)

    def test_resume_reanchors_current_hands_to_measured_robot_like_u_release(self):
        inputs,robots,links = self.capture()
        n=self.node
        saved = signature(n._session)
        n._set_arm_control_enabled(False, 'Y+B')
        now=time.monotonic(); ros=n.get_clock().now().nanoseconds*1e-9
        n._stopped_at,n._stopped_ros=now-1,ros-1
        n._robot_poses={s:(p.copy(),now-.01,ros-.01) for s,p in robots.items()}
        n.left_controller_matrix[0,3] += .2
        n.left_controller_matrix[:3,:3] = Rotation.from_euler('x', 60., degrees=True).as_matrix()
        n._set_arm_control_enabled(True, 'X+A')
        self.assertTrue(n.arm_control_enabled)
        self.assertEqual(n._phase, 'SETTLING')
        self.assertNotEqual(saved, signature(n._session))
        np.testing.assert_allclose(n._session.mapper.target('left',n.left_controller_matrix), links['left']['wrist'])
        self.assertEqual(n._session.position_tolerance_m, .03)
        self.assertEqual(n._session.rotation_tolerance_deg, 15.)

    def test_ready_rezeros_moved_hands_without_changing_robot_targets(self):
        self.capture()
        n = self.node
        goals, basis, templates = deepcopy(n._goals), n._mapper.basis.copy(), deepcopy(n._session.templates)
        for side in ('left', 'right'):
            pose = getattr(n, side+'_controller_matrix')
            pose[:3, 3] += [1.2, -.1, .6]
            pose[:3, :3] = Rotation.from_euler('y', 80., degrees=True).as_matrix() @ pose[:3, :3]
        now = time.monotonic()
        n._phase = 'WARMUP'
        n._enable_at = now-1.
        n._first_command_rx = {s: now-.2 for s in goals}
        n._command_rx = {s: now for s in goals}
        n._startup_reconfirm = True
        n._last_pub = None
        n._publish_synced_pose_frame_if_ready()
        self.assertEqual(n._phase, 'ACTIVE')
        self.assertFalse(n._startup_reconfirm)
        np.testing.assert_array_equal(n._mapper.basis, basis)
        for side in goals:
            np.testing.assert_array_equal(n._goals[side], goals[side])
            np.testing.assert_allclose(n._mapper.target(side, getattr(n, side+'_controller_matrix')), goals[side], atol=1e-12)
            np.testing.assert_array_equal(n._session.templates[side].shoulder, templates[side].shoulder)
            self.assertEqual(n._session.templates[side].maximum, templates[side].maximum)

    def test_wait_start_XA_rezeros_without_restarting_controller(self):
        self.capture()
        n = self.node
        goals = deepcopy(n._goals)
        n._phase = 'WAIT_START'
        n._command_rx = {s: time.monotonic() for s in goals}
        n.left_controller_matrix[0, 3] += .4
        n.right_controller_matrix[:3, :3] = Rotation.from_euler('x', 65., degrees=True).as_matrix()
        with patch.object(n, '_publish_reactivate') as activate:
            n._set_arm_control_enabled(True, 'X+A')
        activate.assert_not_called()
        self.assertTrue(n.arm_control_enabled)
        self.assertEqual(n._phase, 'ACTIVE')
        for side in goals:
            np.testing.assert_array_equal(n._goals[side], goals[side])
            np.testing.assert_allclose(n._mapper.target(side, getattr(n, side+'_controller_matrix')), goals[side], atol=1e-12)

    def test_rezero_rejects_partial_invalid_frame_atomically(self):
        inputs, actual, _ = self.capture()
        before = signature(self.node._session)
        inputs['left'][0, 3] += .5
        inputs['right'][0, 0] = float('nan')
        with self.assertRaises(ValueError):
            self.node._rezero_start_inputs(inputs, actual, 'invalid frame')
        self.assertEqual(signature(self.node._session), before)

    def test_rezero_does_not_hide_moved_robot(self):
        inputs, actual, _ = self.capture()
        before = signature(self.node._session)
        actual['left'][0, 3] += .06
        with self.assertRaises(ValueError):
            self.node._rezero_start_inputs(inputs, actual, 'robot moved')
        self.assertEqual(signature(self.node._session), before)

    def test_startup_timing_uses_controller_parameters(self):
        n = self.node
        n._hardware_manifest = {'urdf': '/test.urdf', 'srdf': '/test.srdf'}
        from hardware.xr_contract import private_topic
        values = [SimpleNamespace(string_value=s) for s in
                  ('/test.urdf', '/test.srdf', '', '', '', 'arm_r_link7', 'arm_l_link7',
                   private_topic('sg2', 'right'), private_topic('sg2', 'left'))]
        for index, value in ((2, 0.), (3, 10.), (4, 1.)):
            values[index] = SimpleNamespace(type=3, double_value=value)
        values += [SimpleNamespace(type=3, double_value=1.), SimpleNamespace(type=3, double_value=.8)]
        n._profile_result(SimpleNamespace(result=lambda: SimpleNamespace(values=values)))
        self.assertTrue(n._profile_verified)
        self.assertEqual(n.CONTROLLER_ACTIVATION_DELAY, 1.)
        self.assertEqual(n.WARMUP_HOLD_SECONDS, .8)

    def test_SH5_operating_profile_requires_zero_elbow_weight(self):
        n=self.node;n.hardware_model='sh5'
        n._hardware_manifest={'urdf':'/test.urdf','srdf':'/test.srdf','arm_limit_policy':'sh5_bent_elbow_v1'}
        from hardware.xr_contract import private_topic
        values=[SimpleNamespace(string_value=s) for s in ('/test.urdf','/test.srdf','','','','arm_r_link7','arm_l_link7',private_topic('sh5','right'),private_topic('sh5','left'))]
        for i,x in ((2,0.),(3,10.),(4,1.)):values[i]=SimpleNamespace(type=3,double_value=x)
        values += [SimpleNamespace(type=3,double_value=0.),SimpleNamespace(type=3,double_value=.03)]
        future=SimpleNamespace(result=lambda:SimpleNamespace(values=values))
        n._profile_result(future);self.assertTrue(n._profile_verified)
        values[2].double_value=.1;n._profile_result(future);self.assertFalse(n._profile_verified)

    def test_remote_epoch_joint_samples_are_fresh_only_by_receipt(self):
        n=self.node
        names=[f'arm_{s}_joint{i}' for s in ('l','r') for i in range(1,8)]
        msg=JointState(name=names,position=[0.]*14)
        msg.header.stamp.sec=10  # Robot boot clock, unrelated to PC epoch.
        n.joint_states_callback(msg)
        self.assertIsNotNone(n._joint_rx)
        first=n._joint_rx
        n.joint_states_callback(msg)
        self.assertEqual(n._joint_rx,first)
        n._stopped_at=time.monotonic()-.5
        now=time.monotonic()
        n._joint_history.clear()
        n._joint_history.extend([(now-.4,10.,np.zeros(14)), (now-.2,10.1,np.zeros(14)),
                                 (now-.1,10.2,np.zeros(14))])
        n._stopped_ros=1e12
        XRResumePublisher._require_still_robot(n,now)

    def test_tf_receipt_watchdog_not_refreshed_by_cached_lookup(self):
        n=self.node
        transform=TransformStamped(child_frame_id='arm_l_link7')
        transform.header.stamp.sec=10
        transform.transform.rotation.w=1.
        with patch('time.monotonic',return_value=2.):
            n._tf_received(TFMessage(transforms=[transform]))
        with patch.object(n.tf_buffer,'lookup_transform',return_value=transform):
            with patch('time.monotonic',return_value=2.2):
                n._tf_pose('arm_l_link7')
                n._tf_received(TFMessage(transforms=[transform]))  # duplicate does not refresh
            with patch('time.monotonic',return_value=2.+n.FEEDBACK_MAX_AGE+.1):
                with self.assertRaisesRegex(ValueError,'TF updates stopped'):
                    n._tf_pose('arm_l_link7')

    def test_boot_epoch_pose_order_survives_manual_stop(self):
        n=self.node
        msg=PoseStamped()
        msg.header.frame_id='base_link';msg.header.stamp.sec=10;msg.pose.orientation.w=1.
        n._robot_pose(msg,'left')
        self.assertIn('left',n._robot_poses)
        n._set_arm_control_enabled(False,'Y+B')
        n._robot_pose(msg,'left')
        self.assertNotIn('left',n._robot_poses)
        msg.header.stamp.sec=11
        n._robot_pose(msg,'left')
        self.assertIn('left',n._robot_poses)

    def test_gate_status_accepts_clock_offset_but_rejects_reordered_sequence(self):
        n=self.node
        def packet(sequence,epoch):
            return String(data=json.dumps({'model':'sg2','readiness_policy':n.hardware_readiness_policy,
                'gate_instance':'gate','status_sequence':sequence,'ros_time_s':epoch,
                'description_ok':True,'motor_healthy':True,'measured_fresh':True,'fault':None}))
        n._gate_status(packet(1,10.))
        self.assertGreater(n._gate_rx,0.)
        n._gate_rx=-100.
        n._gate_status(packet(1,1e12))
        self.assertEqual(n._gate_rx,-100.)
        n._gate_status(packet(2,0.))
        self.assertGreater(n._gate_rx,0.)

    def test_far_old_target_bridges_with_gate_disabled_before_measured_resume(self):
        _,robots,_=self.capture()
        n=self.node
        n._last_published=deepcopy(robots)
        n._last_published['left'][0,3]+=.3
        n._set_arm_control_enabled(False,'Y+B')
        now=time.monotonic()
        n._stopped_at=now-1
        n._robot_poses={s:(p.copy(),now-.01,10.) for s,p in robots.items()}
        n.left_controller_matrix[0,3]+=.2
        packets=[]
        class Sink:
            def publish(self,msg): packets.append(json.loads(msg.data))
        n._intent_pub=Sink()
        n._set_arm_control_enabled(True,'X+A')
        self.assertEqual(n._phase,'RESUME_SEED')
        self.assertFalse(packets[-1]['enabled'])
        expected=deepcopy(n._resume_bridge_targets)
        base=time.monotonic()
        for i in range(400):
            with patch('time.monotonic',return_value=base+i/30):
                previous=deepcopy(n._goals)
                n._publish_synced_pose_frame_if_ready()
                self.assertLessEqual(np.linalg.norm(n._goals['left'][:3,3]-previous['left'][:3,3]),min(n.goal_speed/30,.1)+1e-8)
                if n._phase=='SETTLING': break
                self.assertFalse(packets[-1]['enabled'])
        self.assertEqual(n._phase,'SETTLING')
        self.assertIsNone(n._resume_bridge_targets)
        self.assertTrue(packets[-1]['enabled'])
        np.testing.assert_array_equal(n._goals['left'],expected['left'])

    def auxiliary_event(self, left_stick, right_stick, clicked=False, left_trigger=0., right_trigger=0.):
        n=self.node
        inputs,_,_=self.capture()
        n._phase='ACTIVE'
        n._publish_synced_pose_frame_if_ready=lambda: None
        n._hardware_manifest={'auxiliary_limits': aux_fixture()}
        n.head_joint1_current_position=.2
        n.head_joint2_current_position=.1
        n.lift_joint_current_position=-.2
        data = {s: inputs[s].flatten(order='F').tolist() for s in inputs}
        data['leftState']={'thumbstickValue':left_stick,'thumbstick':clicked,'triggerValue':left_trigger}
        data['rightState']={'thumbstickValue':right_stick,'thumbstick':clicked,'triggerValue':right_trigger}
        asyncio.run(n.on_controller_move(SimpleNamespace(value=data), None))

    def test_sticks_and_triggers_route_into_single_gate_lease(self):
        self.auxiliary_event([1., .5], [0., 1.], left_trigger=1., right_trigger=.5)
        n=self.node
        self.assertIn('head_joint1', n._aux_joints)
        self.assertIn('lift_joint', n._aux_joints)
        self.assertEqual(n._aux_joints['gripper_l_joint1'], 1.1)
        self.assertAlmostEqual(n._aux_joints['gripper_r_joint1'], .55)
        self.assertEqual(n._aux_base, [0., 0., 0.])
        self.assertEqual(n.count_publishers('/cmd_vel'), 0)
        self.assertEqual(n.count_publishers('/leader/joystick_controller_left/joint_trajectory'), 0)
        packets=[]
        class Sink:
            def publish(self, msg): packets.append(json.loads(msg.data))
        n._intent_pub=Sink()
        n._heartbeat()
        self.assertEqual(packets[-1]['aux']['joints'], n._aux_joints)
        n._set_arm_control_enabled(False, 'Y+B')
        self.assertEqual(packets[-1]['aux'], {})
        self.assertEqual(n._aux_joints, {})

    def test_both_clicks_restore_mobile_mode_with_lift(self):
        self.auxiliary_event([1., .5], [.6, 1.], clicked=True)
        self.assertFalse(self.node.joystick_mode)
        self.assertTrue(any(self.node._aux_base))
        self.assertIn('lift_joint', self.node._aux_joints)
        self.assertNotIn('head_joint1', self.node._aux_joints)

    def test_head_and_lift_increment_from_measured_feedback(self):
        msg = JointState(name=[k for s in ('l','r') for k in [f'arm_{s}_joint{i}' for i in range(1,8)]]+
                        ['head_joint1','head_joint2','lift_joint'], position=[0.]*14+[.3,-.1,-.25])
        msg.header.stamp = self.node.get_clock().now().to_msg()
        self.node.joint_states_callback(msg)
        self.assertEqual(self.node.head_joint1_current_position, .3)
        self.assertEqual(self.node.head_joint2_current_position, -.1)
        self.assertEqual(self.node.lift_joint_current_position, -.25)
        msg.position = list(msg.position[:-3]) + [.5, .2, -.4]
        self.node.joint_states_callback(msg)
        self.assertEqual(self.node.head_joint1_current_position, .3)
        self.assertEqual(self.node.lift_joint_current_position, -.25)

    def test_measured_lift_moves_workspace_without_recalibration(self):
        _,_,links=self.capture()
        n=self.node
        # Exercise the real snapshot override with synthetic measured TF poses.
        n._snapshot=HardwareXRPublisher._snapshot.__get__(n)
        old_basis=n._session.mapper.basis.copy()
        old_input=n._session.mapper.anchors['left'].input_pose.copy()
        old_goal=n._goals['left'].copy()
        shift=np.array([0.,0.,.04])
        moved=deepcopy(links)
        for group in moved.values():
            for pose in group.values(): pose[:3,3]+=shift
        with patch.object(XRDeltaPublisher, '_snapshot', return_value=moved):
            n._snapshot()
            n._snapshot()
        np.testing.assert_allclose(n._goals['left'][:3,3], old_goal[:3,3]+shift)
        np.testing.assert_array_equal(n._session.mapper.basis, old_basis)
        np.testing.assert_array_equal(n._session.mapper.anchors['left'].input_pose, old_input)


    def test_SH5_measured_lift_transports_stationary_wrist_in_both_input_modes(self):
        # [ADDED] Optical hands and mode-2 controllers share this exact
        # HardwareXRPublisher snapshot method: no VR hand displacement required.
        _, _, links = self.capture()
        n = self.node
        n.hardware_model = 'sh5'
        n._snapshot = HardwareXRPublisher._snapshot.__get__(n)
        original_goal = n._goals['left'][:3,3].copy()
        original_anchor = n._session.mapper.anchors['left'].robot_pose[:3,3].copy()
        original_input = n._session.mapper.anchors['left'].input_pose.copy()
        original_shoulder = n._session.templates['left'].shoulder.copy()
        unchanged_tf = deepcopy(links)  # SH5 lift joint feedback drives compensation.
        with patch.object(XRDeltaPublisher, '_snapshot', return_value=unchanged_tf):
            n.lift_joint_current_position = .06
            n._snapshot()
            n._snapshot()  # [DEBUG] A repeated snapshot must never double-add.
            np.testing.assert_allclose(n._goals['left'][:3,3],
                                       original_goal + [0.,0.,.06])
            np.testing.assert_allclose(n._session.mapper.anchors['left'].robot_pose[:3,3],
                                       original_anchor + [0.,0.,.06])
            np.testing.assert_allclose(n._session.templates['left'].shoulder,
                                       original_shoulder + [0.,0.,.06])
            np.testing.assert_array_equal(n._session.mapper.anchors['left'].input_pose,
                                          original_input)
            n.lift_joint_current_position = -.04
            n._snapshot()
            np.testing.assert_allclose(n._goals['left'][:3,3],
                                       original_goal + [0.,0.,-.04])

    def test_no_reference_change_on_repeated_active_XA(self):
        self.capture()
        before=signature(self.node._session)
        self.node._phase='ACTIVE'
        for _ in range(3): self.node._set_arm_control_enabled(True,'active X+A')
        self.assertEqual(before,signature(self.node._session))

    def test_gate_fault_is_not_misreported_as_cyclo_reference_fault(self):
        # [ADDED 2026-10-14] Exact source routing of the user's joint6 failure.
        self.capture()
        before = signature(self.node._session)
        self.node._hardware_gate_fault(Bool(data=True))
        self.assertFalse(self.node.arm_control_enabled)
        self.assertTrue(self.node._controller_fault_latched)
        self.assertIn('hardware gate fault', self.node._last_reason)
        self.assertNotIn('Cyclo reference fault', self.node._last_reason)
        self.assertEqual(before, signature(self.node._session))
        reason = 'arm_l_joint6: commanded/measured error exceeds .08rad'
        msg = String(data=json.dumps({
            'model': 'sg2', 'readiness_policy': self.node.hardware_readiness_policy,
            'ros_time_s': self.node.get_clock().now().nanoseconds * 1e-9,
            'fault': reason, 'reason': reason,
            'gate_instance': 'test-gate', 'status_sequence': 1,
        }))
        self.node._gate_status(msg)
        self.assertEqual(self.node._gate_fault_reason, reason)
        self.assertIn(reason, self.node._last_reason)
        with patch.object(self.node, '_event') as event:
            self.node._gate_status(msg)
            self.node._hardware_gate_fault(Bool(data=True))
            event.assert_not_called()
        self.assertEqual(before, signature(self.node._session))

    def test_actual_reference_checker_fault_still_stops(self):
        self.capture()
        self.node._reference_fault(Bool(data=True))
        self.assertFalse(self.node.arm_control_enabled)
        self.assertTrue(self.node._controller_fault_latched)
        self.assertIn('Cyclo reference fault', self.node._last_reason)

    def test_manual_fault_reset_waits_for_ordered_ack_then_restarts(self):
        _,robots,_=self.capture()
        n=self.node
        n._hardware_gate_fault(Bool(data=True))
        self.assertTrue(n._controller_fault_latched)
        class Pending:
            def add_done_callback(self,cb): self.callback=cb
            def result(self):
                return Trigger.Response(success=True,message=json.dumps({'gate_instance':'g','status_sequence':7}))
            def cancel(self): pass
        pending=Pending()
        n._fault_reset_client=SimpleNamespace(service_is_ready=lambda:True,call_async=lambda req:pending)
        n._set_arm_control_enabled(True,'X+A')
        self.assertFalse(n.arm_control_enabled)
        self.assertIs(n._fault_reset_future,pending)
        n._fault_reset_result(pending)
        self.assertTrue(n._controller_fault_latched)
        now=time.monotonic()
        n._stopped_at=now-1
        n._robot_poses={s:(p.copy(),now-.01,10.) for s,p in robots.items()}
        data={'model':'sg2','readiness_policy':n.hardware_readiness_policy,'gate_instance':'g',
              'status_sequence':8,'ros_time_s':10.,'fault':None,'description_ok':True,
              'motor_healthy':True,'measured_fresh':True}
        n._gate_status(String(data=json.dumps(data)))
        self.assertFalse(n._controller_fault_latched)
        self.assertTrue(n.arm_control_enabled)
        self.assertEqual(n._phase,'SETTLING')
        n._hardware_gate_fault(Bool(data=True))  # old unsequenced Bool cannot re-latch
        self.assertTrue(n.arm_control_enabled)
        data['status_sequence']=9;data['fault']='new real fault'
        n._gate_status(String(data=json.dumps(data)))
        self.assertTrue(n._controller_fault_latched)
        self.assertFalse(n.arm_control_enabled)


if __name__ == '__main__': unittest.main()

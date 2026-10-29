import ast,json,time,unittest
from pathlib import Path
from types import SimpleNamespace as S
from hardware.sh5_controller_hands import SH5HandOutput,targets
from hardware.controller_hand_modes import ControllerHandModes
ROOT=Path(__file__).resolve().parents[1]
class Mode1Policy(unittest.TestCase):
 def test_mode1_grip_preserved(self):
  node=SH5HandOutput.__new__(SH5HandOutput);node.hand_mode=1
  values={s:dict(trigger=.8,grip=.6) for s in ('left','right')}
  node.receive(values,1.)
  poses=json.loads((ROOT/'hardware/hx5_controller_poses.json').read_text())['poses']
  original={s:dict(trigger=.8,grip=.6) for s in values}
  self.assertEqual(targets(poses,node.controls),targets(poses,original))
  self.assertEqual(node.controls['left']['grip'],.6)
 def test_hold_sticks_require_explicit_hold_freshness_and_no_fault(self):
  path=ROOT/'robotis_vuer/robotis_vuer/vr_publisher_sh5_hands_hardware.py'
  tree=ast.parse(path.read_text());fn=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='_hold_sticks_ready')
  ns={};exec(compile(ast.Module(body=[fn],type_ignores=[]),str(path),'exec'),ns)
  n=S(_controller_mode=True,_hand_mode=1,hardware_allow_arm=True,_hold_sticks_enabled=True,arm_control_enabled=False,_ready_rehome_pending=False,_controller_fault_latched=False,_hardware_manifest={'manual_auxiliary':True},_controller_event_rx=10.,_joint_rx=10.,_gate_rx=10.,_gate_state={'fault':None},_tracking_input_state=lambda now:'fresh')
  check=lambda:ns['_hold_sticks_ready'](n,10.1)
  self.assertTrue(check())
  for key,value in [('_hand_mode',2),('_hold_sticks_enabled',False),('_ready_rehome_pending',True),('_controller_fault_latched',True),('_joint_rx',9.),('_gate_rx',9.),('_controller_event_rx',9.),('arm_control_enabled',True),('_gate_state',{'fault':'collision'})]:
   old=getattr(n,key);setattr(n,key,value);self.assertFalse(check(),key);setattr(n,key,old)

 def test_hold_intent_keeps_head_and_base_but_no_hands_or_lift(self):
  import numpy as np
  from hardware.controller_hand_modes import POSE_CODES
  path=ROOT/'robotis_vuer/robotis_vuer/vr_publisher_sh5_hands_hardware.py'
  fn=next(n for n in ast.walk(ast.parse(path.read_text())) if isinstance(n,ast.FunctionDef) and n.name=='_extra_intent')
  ns=dict(np=np,SIDES=('left','right'),POSE_CODES=POSE_CODES,time=time)
  exec(compile(ast.Module(body=[fn],type_ignores=[]),str(path),'exec'),ns)
  n=S(_controller_mode=True,_hand_mode=1,_controller_event_rx=10.,_tracking_input_state=lambda now:'fresh',
      arm_control_enabled=False,_phase='HOLD',_controller_fault_latched=False,
      left_controller_state={'triggerValue':.8,'squeezeValue':.6},right_controller_state={'triggerValue':.8,'squeezeValue':.6},
      calibrate_trigger=lambda side,x:x,_hold_sticks_ready=lambda now:True,
      _aux_joints={'head_joint1':.1,'head_joint2':.2,'lift_joint':.3},_aux_base=[.1,0.,.2])
  result=ns['_extra_intent'](n,10.1)
  self.assertTrue(result['manual_enabled']);self.assertIsNone(result['controller_hands'])
  self.assertEqual(result['manual_aux'],{'joints':{'head_joint1':.1,'head_joint2':.2},'base':[.1,0.,.2]})
  self.assertGreater(result['recording_epoch_us'],0)
  n.arm_control_enabled=True;n._phase='ACTIVE';n._hold_sticks_ready=lambda now:False
  active=ns['_extra_intent'](n,10.1)
  self.assertFalse(active['manual_enabled'])
  self.assertEqual(active['controller_hands']['left'],dict(trigger=.8,grip=.6))

if __name__=='__main__':unittest.main()


"""Dynamic tactile parsing uses actual sensor interfaces and never fills missing data."""
from types import SimpleNamespace as S
import ast
from pathlib import Path
from unittest.mock import Mock
import unittest

from hardware.hx5_tactile import (SENSOR_NAMES, PRESSURE_NAMES, dynamic_hand_message,
                                 parse_pressures, TactileReadings)


def sample():
    names=[name for side in ('right','left') for name in reversed(SENSOR_NAMES[side])]
    return S(header=S(stamp=S(sec=12,nanosec=0)),joint_names=names,
             interface_values=[S(interface_names=list(reversed(PRESSURE_NAMES)),
                                 values=[float(v) for v in reversed(range(9))]) for _ in names])


class DynamicTactileTests(unittest.TestCase):
    def test_ros_fallback_calls_validated_ingest_and_prefers_recent_typed_frames(self):
        path=Path(__file__).with_name('loop_vr_streamer.py')
        tree=ast.parse(path.read_text())
        method=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='receive_dynamic_tactile')
        scope=dict(time=S(monotonic=lambda:2.),PRESSURE_TOPICS=dict(left=1,right=2),dynamic_hand_message=dynamic_hand_message)
        exec(compile(ast.fix_missing_locations(ast.Module(body=[method],type_ignores=[])),str(path),'exec'),scope)
        node=S(_typed_tactile_rx={'left':1.9},_accept_tactile=Mock())
        scope['receive_dynamic_tactile'](node,sample())
        self.assertEqual(node._accept_tactile.call_count,1)
        side,msg=node._accept_tactile.call_args.args
        self.assertEqual(side,'right');self.assertEqual(parse_pressures(side,msg),tuple(range(9))*5)

    def test_named_readings_are_sorted_and_serialized_as_uint8(self):
        msg=sample()
        readings=TactileReadings()
        for side in ('left','right'):
            pressure=dynamic_hand_message(side,msg)
            self.assertEqual(parse_pressures(side,pressure),tuple(range(9))*5)
            self.assertTrue(readings.receive(side,pressure,1.))
        payload=readings.payload('ffw_sh5_vr',1.1)
        self.assertEqual(payload['ffw_sh5_vr.observation.left_hand.tactile_pressure'],list(range(9))*5)
        self.assertIsNone(readings.payload('ffw_sh5_vr',2.)['ffw_sh5_vr.observation.left_hand.tactile_pressure'])

    def test_missing_or_malformed_sensor_is_not_zero_filled(self):
        msg=sample();msg.joint_names.pop();msg.interface_values.pop()
        with self.assertRaises(ValueError):dynamic_hand_message('left',msg)
        for value in (256.,-.1,1.5,float('nan')):
            msg=sample();msg.interface_values[0].values[0]=value
            with self.assertRaises(ValueError):dynamic_hand_message('right',msg)

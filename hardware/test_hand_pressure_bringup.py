"""Pressure integration must share hardware and claim no command interfaces."""
from copy import deepcopy
from types import SimpleNamespace
import unittest

import yaml

from hardware.hand_pressure_bringup import CONFIG, CONTROLLERS, PLUGIN, validate_controllers
from hardware.hx5_tactile import PRESSURE_NAMES, PRESSURE_TOPICS, SENSOR_NAMES, parse_pressures


def active_controllers():
    return [SimpleNamespace(name=name, type=PLUGIN, state='active',
                            claimed_interfaces=[], required_command_interfaces=[],
                            required_state_interfaces=[sensor + '/' + key
                                for sensor in SENSOR_NAMES[side] for key in PRESSURE_NAMES])
            for side, name in CONTROLLERS.items()]


class PressureBringupTests(unittest.TestCase):
    def test_generated_ros_uint8_array_is_accepted_and_normalized(self):
        try:
            from robotis_interfaces.msg import HandPressures, TactileSensor
        except ImportError:
            self.skipTest('Run inside ai_worker for generated ROS messages')
        message = HandPressures(hand_name='right', sensors=[
            TactileSensor(sensor_name=name, pressure_names=list(PRESSURE_NAMES),
                          pressure_values=[0, 49, 65, 0, 26, 34, 0, 0, 255])
            for name in SENSOR_NAMES['right']])
        parsed = parse_pressures('right', message)
        self.assertEqual(parsed[:9], (0, 49, 65, 0, 26, 34, 0, 0, 255))
        self.assertTrue(all(type(value) is int for value in parsed))

    def test_config_has_exact_hand_sensor_mapping(self):
        config = yaml.safe_load(CONFIG.read_text())
        for side, name in CONTROLLERS.items():
            params = config[name]['ros__parameters']
            self.assertEqual(params['sensor_names'], list(SENSOR_NAMES[side]))
            self.assertEqual(params['interface_names'], list(PRESSURE_NAMES))
            self.assertEqual(params['hand_name'], side)
            self.assertEqual(params['topic_name'], PRESSURE_TOPICS[side])
            self.assertEqual(params['type'], PLUGIN)  # Spawner resolves type from this node's params.
            self.assertEqual(config['controller_manager']['ros__parameters'][name]['type'], PLUGIN)

    def test_active_broadcasters_are_reused(self):
        self.assertEqual(validate_controllers(active_controllers(), require_active=True), [])

    def test_only_missing_or_inactive_broadcaster_is_started(self):
        controllers = active_controllers()
        self.assertEqual(validate_controllers(controllers[:1]), ['pressure_r_broadcaster'])
        controllers[1].state = 'inactive'
        self.assertEqual(validate_controllers(controllers), ['pressure_r_broadcaster'])
        with self.assertRaises(RuntimeError):
            validate_controllers(controllers, require_active=True)

    def test_command_ownership_is_rejected(self):
        for field in ('claimed_interfaces', 'required_command_interfaces'):
            controllers = active_controllers()
            setattr(controllers[0], field, ['joint/position'])
            with self.assertRaisesRegex(RuntimeError, 'state-only'):
                validate_controllers(controllers)

    def test_wrong_plugin_or_incomplete_sensor_states_is_rejected(self):
        for field, value in [('type', 'joint_trajectory_controller/JointTrajectoryController'),
                             ('required_state_interfaces', ['finger_l_sensor1/Present Pressure 1'])]:
            controllers = deepcopy(active_controllers())
            setattr(controllers[0], field, value)
            with self.assertRaises(RuntimeError):
                validate_controllers(controllers)


if __name__ == '__main__':
    unittest.main()

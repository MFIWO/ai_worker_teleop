"""Attach state-only pressure broadcasters to the existing SH5 controller manager.

Never starts a hardware driver, joint trajectory, or second serial connection.
"""
from pathlib import Path
import subprocess
import time

from hardware.hx5_tactile import PRESSURE_NAMES, SENSOR_NAMES

PLUGIN = 'robotis_hand_pressure_broadcaster/PressureBroadcaster'
CONTROLLERS = {'left': 'pressure_l_broadcaster', 'right': 'pressure_r_broadcaster'}
CONFIG = Path(__file__).with_name('hand_pressure_broadcasters.yaml')


def validate_controllers(controllers, require_active=False):
    by_name = {item.name: item for item in controllers}
    pending = []
    for side, name in CONTROLLERS.items():
        item = by_name.get(name)
        if item is None:
            pending.append(name)
            continue
        if item.type != PLUGIN or item.claimed_interfaces or item.required_command_interfaces:
            raise RuntimeError(f'{name}: expected the official state-only pressure broadcaster')
        if item.state != 'active':
            pending.append(name)
            continue
        expected = {sensor + '/' + key for sensor in SENSOR_NAMES[side] for key in PRESSURE_NAMES}
        if set(item.required_state_interfaces) != expected:
            raise RuntimeError(f'{name}: expected all 45 named pressure interfaces')
    if require_active and pending:
        raise RuntimeError('Pressure broadcasters did not activate: ' + ', '.join(pending))
    return pending


def bringup():
    import rclpy
    from controller_manager_msgs.srv import ListControllers, ListControllerTypes
    from robotis_interfaces.msg import HandPressures
    from hardware.hx5_tactile import PRESSURE_TOPICS, parse_pressures

    rclpy.init(args=[])
    node = rclpy.create_node('xr_sh5_pressure_bringup')
    def call(service, kind):
        client = node.create_client(kind, '/controller_manager/' + service)
        try:
            if not client.wait_for_service(timeout_sec=15.):
                raise RuntimeError('SH5 controller manager unavailable: ' + service)
            future = client.call_async(kind.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=15.)
            if not future.done() or future.result() is None:
                raise RuntimeError('SH5 controller manager did not answer: ' + service)
            return future.result()
        finally:
            node.destroy_client(client)

    try:
        pending = validate_controllers(call('list_controllers', ListControllers).controller)
        if pending:
            if PLUGIN not in call('list_controller_types', ListControllerTypes).types:
                raise RuntimeError('Existing follower cannot discover the pressure plugin; '
                                   'restart the robot session with --bringup to select its overlay')
            subprocess.run(['ros2', 'run', 'controller_manager', 'spawner', *pending,
                            '--controller-manager', '/controller_manager',
                            '--param-file', str(CONFIG), '--controller-manager-timeout', '15',
                            '--service-call-timeout', '15', '--switch-timeout', '15'],
                           check=True, timeout=90)
        validate_controllers(call('list_controllers', ListControllers).controller,
                             require_active=True)
        received = {}
        errors = {}
        def receive(side, message):
            try:
                received[side] = parse_pressures(side, message)
            except (ValueError, TypeError, AttributeError) as error:
                errors[side] = str(error)
        subscriptions = [node.create_subscription(HandPressures, PRESSURE_TOPICS[side],
                          lambda msg, side=side: receive(side, msg), 10)
                         for side in CONTROLLERS]
        deadline = time.monotonic() + 5.
        while len(received) < 2 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        if len(received) < 2:
            raise RuntimeError(f'Missing valid live hand pressure messages: {errors}; '
                               f'received={list(received)}')
        print('[HX5 PRESSURE] Both official state-only broadcasters ACTIVE; '
              '45 uint8 readings per hand received. Zero is valid when unloaded.', flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    bringup()

"""Read-only validation for controller-mode robot ready-pose recovery.

[ADDED 2026-10-28] No ROS import or motor writes. The robot session owns
process shutdown and action execution; PC only issues a request.
"""
import json
import math


def decode_request(raw):
    data = json.loads(raw)
    if (not isinstance(data, dict) or set(data) != {'request_id', 'action'}
            or data['action'] != 'ready_pose' or not isinstance(data['request_id'], str)
            or not 1 <= len(data['request_id']) <= 80
            or not data['request_id'].isascii()
            or not data['request_id'].isalnum()):
        raise ValueError('invalid mode-2 ready-pose request')
    return data['request_id']


def gate_disarmed(status, received_at, now, *, maximum_age=0.5):
    """Require current measured, healthy, explicitly disarmed robot gate."""
    if not isinstance(status, dict) or not math.isfinite(received_at):
        return False
    if not 0. <= now - received_at <= maximum_age:
        return False
    return (status.get('model') == 'sh5'
            and isinstance(status.get('gate_instance'), str)
            and bool(status['gate_instance'])
            and status.get('enabled') is False
            and status.get('forwarding') is False
            and status.get('manual_active') is False
            and not status.get('fault')
            and status.get('description_ok') is True
            and status.get('measured_fresh') is True
            and status.get('motor_healthy') is True)


def recovery_event(request_id, state, message=''):
    """One bounded, ID-matched acknowledgement for the requesting PC."""
    if state not in ('WAIT_DISARM', 'MOVING', 'RESTARTING', 'READY', 'FAILED'):
        raise ValueError('invalid ready recovery state')
    if not isinstance(request_id, str) or not request_id:
        raise ValueError('invalid recovery ID')
    return json.dumps({'request_id': request_id, 'state': state, 'message': str(message)},
                      allow_nan=False)

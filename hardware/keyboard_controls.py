"""Pure Loop keyboard mapping; commands are relayed by the hardware gate."""
import math

MOTION_KEYS = frozenset(('w', 'a', 's', 'd', 'up', 'down', 'left', 'right', 'u', 'i', 'o', 'p'))
# ffw_lg2_leader_ai_hardware_controller.yaml: sensorxel_r_joy_jog_scale.
LEADER_LIFT_JOG_M = .01


def keyboard_aux(keys, measured, limits, previous, dt):
    keys = set(keys)
    if keys - MOTION_KEYS:
        raise ValueError('unknown operator key')
    dt = max(0., min(float(dt), .1))
    x, y = .15 * (('w' in keys)-('s' in keys)), .15 * (('a' in keys)-('d' in keys))
    norm = math.hypot(x, y)
    if norm > .15:
        x, y = x*.15/norm, y*.15/norm
    base = [x, y, .3*(('o' in keys)-('p' in keys))]
    # Head joint1: positive is down; joint2: positive is left.
    directions = {'head_joint1': ('down' in keys)-('up' in keys),
                  'head_joint2': ('left' in keys)-('right' in keys),
                  'lift_joint': ('u' in keys)-('i' in keys)}
    targets = {}
    for name, direction in directions.items():
        if not direction:
            continue
        actual = float(measured[name])
        bound = limits[name]
        if not math.isfinite(actual):
            raise ValueError('nonfinite measured auxiliary joint')
        if name == 'lift_joint':
            # Match native full-deflection leader: measured position +/-10 mm
            # each 100 Hz update. The gate retains physical position/slew checks.
            targets[name] = min(bound['upper'], max(bound['lower'], actual+direction*LEADER_LIFT_JOG_M))
            continue
        speed, lead = .25, .05
        speed = min(speed, bound['velocity'])
        old = min(actual+lead, max(actual-lead, previous.get(name, actual)))
        targets[name] = min(bound['upper'], max(bound['lower'], old+direction*speed*dt))
    return {'joints': targets, 'base': base}

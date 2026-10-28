"""Read-only HX5-D20 pressure parsing for the separate typed Loop Tactile source.

No ROS, serial access, baseline subtraction or hand command publishers here.
The canonical ordering follows ROBOTIS sensor/interface names, never arrival
order. A pressure value is an unsigned raw count, not a calibrated force.
"""
from collections import deque
import math


HAND_MODEL = 'HX5-D20'
HAND_SIDES = ('left', 'right')
PRESSURE_TOPICS = {side: f'/{side}_hand/finger_pressures' for side in HAND_SIDES}
SENSOR_NAMES = {
    side: tuple(f'finger_{letter}_sensor{i}' for i in range(1, 6))
    for side, letter in (('left', 'l'), ('right', 'r'))
}
PRESSURE_NAMES = tuple(f'Present Pressure {i}' for i in range(1, 10))



def parse_pressures(side, message):
    """Validate one complete hand and return its 45 values in stable name order."""
    hand_name = message.hand_name
    if hand_name not in ('', side):
        raise ValueError(f'hand_name={hand_name!r}, expected {side!r}')
    sensors = {}
    for sensor in message.sensors:
        name = sensor.sensor_name
        if name not in SENSOR_NAMES[side] or name in sensors:
            raise ValueError(f'unknown or duplicate tactile sensor {name!r}')
        names, values = tuple(sensor.pressure_names), tuple(sensor.pressure_values)
        if len(names) != 9 or len(values) != 9 or set(names) != set(PRESSURE_NAMES):
            raise ValueError(f'{name}: expected nine named Present Pressure interfaces')
        if any(isinstance(v, bool) or not isinstance(v, int) or not 0 <= v <= 255 for v in values):
            raise ValueError(f'{name}: raw pressure must be uint8 (0..255)')
        by_name = dict(zip(names, values))
        sensors[name] = tuple(by_name[key] for key in PRESSURE_NAMES)
    if set(sensors) != set(SENSOR_NAMES[side]):
        raise ValueError('expected all five fingertip sensors')
    return tuple(value for name in SENSOR_NAMES[side] for value in sensors[name])


class TactileReadings:
    """Latest validated readings; stale/malformed data is absent, never zero-filled."""
    def __init__(self, freshness=.5):
        if not math.isfinite(freshness) or freshness <= 0:
            raise ValueError('tactile freshness must be finite and positive')
        self.freshness = freshness
        self.receipts = {side: None for side in HAND_SIDES}
        self.samples = {side: None for side in HAND_SIDES}
        self.header_stamps = {side: -1. for side in HAND_SIDES}
        self.sequences = {side: 0 for side in HAND_SIDES}
        self.history = {side: deque(maxlen=256) for side in HAND_SIDES}
        self.errors = {side: '' for side in HAND_SIDES}

    def receive(self, side, message, now):
        self.receipts[side] = now
        self.sequences[side] += 1
        history = self.history[side]
        history.append(now)
        while len(history) > 1 and now-history[0] > 2.:
            history.popleft()
        self.samples[side] = None
        self.header_stamps[side] = -1.
        try:
            values = parse_pressures(side, message)
            stamp = message.header.stamp
            if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
                raise ValueError('invalid tactile ROS header timestamp')
            self.header_stamps[side] = stamp.sec+stamp.nanosec/1e9
            self.samples[side] = values
            self.errors[side] = ''
            return True
        except (ValueError, TypeError, AttributeError) as error:
            self.errors[side] = str(error)
            return False

    def payload(self, robot_type, now):
        result = {}
        for side in HAND_SIDES:
            receipt = self.receipts[side]
            age = -1. if receipt is None else max(0., now-receipt)
            valid = self.samples[side] is not None
            fresh = valid and receipt is not None and 0 <= now-receipt <= self.freshness
            history = self.history[side]
            hz = ((len(history)-1)/(history[-1]-history[0])
                  if fresh and len(history) > 1 and history[-1] > history[0] else 0.)
            prefix = f'{robot_type}.observation.{side}_hand'
            result[prefix+'.tactile_pressure'] = list(self.samples[side]) if fresh else None
            result[prefix+'.tactile_status'] = [float(receipt is not None), float(valid), float(fresh), hz]
            result[prefix+'.tactile_timing'] = [age, self.header_stamps[side], float(self.sequences[side])]
        return result

"""[ADDED 2026-10-28] SH5 controller-stick shaping; pure Python, no ROS/motors.

The physical URDF is authoritative for head position limits. MOBILE permits
one translational axis at a time; yaw is separate, so up cannot mean diagonal.
A mode click is never interpreted as a mobile motion command.
"""
import math


def _axis(value, deadzone):
    if not math.isfinite(value) or abs(value) > 1.5:
        raise ValueError('invalid controller axis')
    magnitude = abs(value)
    if magnitude <= deadzone:
        return 0.0
    return math.copysign(min(1.0, (magnitude - deadzone) / (1.0 - deadzone)), value)


def head_targets(stick, previous, measured, limits, elapsed, speed=0.30):
    """Jog both head joints in rad/s, saturating at the *live URDF* bounds.

    [CHANGED] Avoid the legacy 0.06 rad/event increment: an event rate is not
    a velocity, and 30 Hz can reach the SH5 pan stop in 0.2 seconds.
    """
    if not math.isfinite(elapsed) or elapsed <= 0.0:
        raise ValueError('head elapsed time must be positive')
    if not math.isfinite(speed) or speed <= 0.0:
        raise ValueError('head speed must be positive')
    names = ('head_joint1', 'head_joint2')
    if len(stick) != 2 or len(measured) != 2 or (previous is not None and len(previous) != 2):
        raise ValueError('expected two head axes and two measured positions')
    targets = []
    for index, name in enumerate(names):
        bound = limits[name]
        lower, upper, velocity = (float(bound[k]) for k in ('lower', 'upper', 'velocity'))
        base = float(measured[index] if previous is None else previous[index])
        if not math.isfinite(float(stick[index])) or abs(float(stick[index])) > 1.05:
            raise ValueError(f'{name}: invalid joystick input')
        demand = _axis(float(stick[index]), 0.10)
        if (not all(map(math.isfinite, (lower, upper, velocity, base)))
                or lower >= upper or velocity <= 0):
            raise ValueError(f'{name}: invalid live URDF limits/feedback')
        # [CHANGED] Rate limit is independent of WebXR event frequency.
        step = demand * min(speed, velocity) * min(elapsed, 0.05)
        targets.append(max(lower, min(upper, base + step)))
    return tuple(targets)


class MobileStickFilter:
    """Neutral recenter + exclusive translation + bounded acceleration/reversal.

    Input is the *already mapped* legacy SG2 stick convention:
    left=[forward_axis, side_axis], right=[unused_lift_axis, yaw_axis].
    Velocity output uses the original signs and max scales (.2 m/s, 1/3 rad/s).
    """
    MAX_CENTER = 0.22
    STABLE_ERROR = 0.025
    STABLE_SECONDS = 0.30
    DEADZONE = 0.085
    REVERSE_PAUSE = 0.16
    SPEED = (0.2, 0.2, 1.0 / 3.0)
    ACCEL = (0.65, 0.65, 1.0)
    DECEL = (1.10, 1.10, 1.7)

    def __init__(self):
        self.center = None
        self.disarm('neutral calibration required')

    def disarm(self, reason='mode change / blocked input'):
        """[CHANGED] A click/timeout cancels velocity, not just its filter state."""
        self.ready = False
        self.reason = reason
        self._quiet_reference = None
        self._quiet_since = None
        self._idle_since = None
        self._last_time = None
        self._axis_lock = None
        self._direction = [0, 0, 0]
        self._reverse_since = [None, None, None]
        self.output = (0.0, 0.0, 0.0)

    def update(self, left, right, *, pressed=False, now):
        if (len(left) != 2 or len(right) != 2
                or not math.isfinite(now)):
            self.disarm('invalid stick shape/time')
            raise ValueError('expected two finite values per stick and a finite time')
        try:
            raw = tuple(float(v) for v in (*left, *right))
            if not all(math.isfinite(v) and abs(v) <= 1.05 for v in raw):
                raise ValueError('invalid/nonfinite stick sample')
        except (ValueError, TypeError, OverflowError):
            self.disarm('invalid controller sample')
            raise ValueError('invalid controller sample') from None

        if pressed:
            self.disarm('stick click: released neutral required')
            return self.output

        if self._last_time is not None and now < self._last_time:
            self.disarm('nonmonotonic controller time')
            return self.output

        if not self.ready:
            # [CHANGED] Never learn an obviously deflected stick as "zero".
            if max(abs(v) for v in raw) > self.MAX_CENTER:
                self._quiet_reference = self._quiet_since = None
                self.reason = 'release both sticks to neutral (offset > 0.22)'
            elif (self._quiet_reference is None or
                  max(abs(v-r) for v, r in zip(raw, self._quiet_reference)) > self.STABLE_ERROR):
                self._quiet_reference, self._quiet_since = raw, now
                self.reason = 'waiting for stable neutral'
            elif now - self._quiet_since >= self.STABLE_SECONDS:
                self.center = raw
                self.ready = True
                self._last_time = now
                self.reason = 'neutral calibrated; mobile ready'
            return self.output

        dt = min(0.05, max(0.0, now - self._last_time))
        self._last_time = now
        corrected = tuple(v-c for v, c in zip(raw, self.center))
        x, y, _, yaw = (_axis(v, self.DEADZONE) for v in corrected)

        # [CHANGED] No diagonal / accidental yaw while translating.
        # Lock one axis until both axes actually return to neutral.
        if x == 0.0 and y == 0.0:
            self._axis_lock = None
        elif self._axis_lock is None and max(abs(self.output[0]), abs(self.output[1])) < 1e-5:
            # Do not start the other axis until the previous translation has stopped.
            self._axis_lock = 0 if abs(x) >= abs(y) else 1
        dx = -x * self.SPEED[0] if self._axis_lock == 0 else 0.0
        dy = y * self.SPEED[1] if self._axis_lock == 1 else 0.0
        dw = (-yaw * self.SPEED[2] if self._axis_lock is None
              and max(abs(self.output[0]), abs(self.output[1])) < 1e-5 else 0.0)
        desired = [dx, dy, dw]

        next_output = []
        for index, target in enumerate(desired):
            prior = self.output[index]
            sign = (1 if target > 0 else -1) if abs(target) > 1e-12 else 0
            # [CHANGED] Release bounce cannot briefly command reverse.
            if sign == 0:
                self._reverse_since[index] = None
                if abs(prior) < 1e-5:
                    self._direction[index] = 0
            elif self._direction[index] == 0:
                self._direction[index] = sign
            elif sign != self._direction[index]:
                if self._reverse_since[index] is None:
                    self._reverse_since[index] = now
                if now - self._reverse_since[index] < self.REVERSE_PAUSE:
                    target = 0.0
                else:
                    self._direction[index] = sign
                    self._reverse_since[index] = None
            else:
                self._reverse_since[index] = None
            rate = self.DECEL[index] if abs(target) < abs(prior) or target*prior < 0 else self.ACCEL[index]
            delta = max(-rate*dt, min(rate*dt, target-prior))
            value = prior + delta
            if abs(value) < 1e-5 and abs(target) < 1e-5:
                value = 0.0
            next_output.append(max(-self.SPEED[index], min(self.SPEED[index], value)))
        self.output = tuple(next_output)

        # Slow bounded center tracking only after a long, *already neutral*
        # output. Never adapt center during motion or large stick deflection.
        if (max(abs(v) for v in corrected) < 0.045
                and max(abs(v) for v in self.output) < 1e-5):
            if self._idle_since is None:
                self._idle_since = now
            elif now-self._idle_since > 1.0:
                alpha = min(0.02*dt, 0.001)
                self.center = tuple(c + alpha*(v-c) for v, c in zip(raw, self.center))
        else:
            self._idle_since = None
        self.reason = 'mobile command' if any(self.output) else 'mobile neutral'
        return self.output

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
    # Left stick HEAD follows only its dominant axis, as MOBILE already does.
    dominant = 0 if abs(float(stick[0])) >= abs(float(stick[1])) else 1
    targets = []
    for index, name in enumerate(names):
        if name not in limits:
            raise ValueError(f'{name}: live URDF limit unavailable')
        bound = limits[name]
        lower, upper, velocity = (float(bound[k]) for k in ('lower', 'upper', 'velocity'))
        base = float(measured[index] if previous is None else previous[index])
        if not math.isfinite(float(stick[index])) or abs(float(stick[index])) > 1.05:
            raise ValueError(f'{name}: invalid joystick input')
        demand = _axis(float(stick[index]), 0.10) if index == dominant else 0.0
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
    SPEED = (0.2, 0.2, 1.0 / 3.0)
    ACCEL = (0.65, 0.65, 1.0)
    DECEL = (1.10, 1.10, 1.7)
    # [CHANGED 2026-10-28] Clamp d(v)/dt and d²(v)/dt² (accel/jerk).
    JERK = (12.0, 12.0, 18.0)
    # A reverse direction requires an observed neutral, not just a timer.
    STOP_EPS = 1e-5

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
        self._acceleration = [0.0, 0.0, 0.0]
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

        # [CHANGED] A gap is not a large integration step that should create
        # an unobserved motion. The 300ms cutoff matches the PC intent TTL.
        if now - self._last_time > 0.30:
            self.disarm('stale controller samples')
            return self.output
        dt = min(0.05, max(0.0, now - self._last_time))
        self._last_time = now
        corrected = tuple(v-c for v, c in zip(raw, self.center))
        x, y, _, yaw = (_axis(v, self.DEADZONE) for v in corrected)

        # [CHANGED] No diagonal / accidental yaw while translating.
        # Lock one axis until both axes actually return to neutral.
        if x == 0.0 and y == 0.0:
            self._axis_lock = None
        elif (self._axis_lock is None and
              max(abs(value) for value in self.output) < self.STOP_EPS):
            # [CHANGED] A new translation must wait for old XY *and yaw* to stop.
            self._axis_lock = 0 if abs(x) >= abs(y) else 1
        dx = -x * self.SPEED[0] if self._axis_lock == 0 else 0.0
        dy = y * self.SPEED[1] if self._axis_lock == 1 else 0.0
        dw = (-yaw * self.SPEED[2] if x == 0.0 and y == 0.0
              and self._axis_lock is None
              and max(abs(self.output[0]), abs(self.output[1])) < self.STOP_EPS
              else 0.0)
        desired = [dx, dy, dw]

        next_output = []
        for index, target in enumerate(desired):
            prior = self.output[index]
            sign = (1 if target > 0 else -1) if abs(target) > 1e-12 else 0

            if sign == 0:
                # Neutral releases the direction latch *after* braking to zero.
                if abs(prior) < self.STOP_EPS:
                    self._direction[index] = 0
            elif self._direction[index] == 0:
                self._direction[index] = sign
            elif sign != self._direction[index]:
                # [CHANGED 2026-10-28] Do not authorize a sign reversal just
                # because a negative joystick rebound lasted 160ms. Brake to
                # zero and require an actual neutral sample before reversing.
                target = 0.0

            if dt <= 0.0:
                next_output.append(prior)
                continue

            # [CHANGED] Finite-difference derivative clamp: velocity command
            # rate (acceleration) and its rate of change (jerk), both in SI.
            rate = (self.ACCEL[index] if abs(target) > abs(prior)
                    and target * prior >= 0.0 else self.DECEL[index])
            requested_accel = max(-rate, min(rate, (target - prior) / dt))
            last_accel = self._acceleration[index]
            max_accel_change = self.JERK[index] * dt
            accel = max(last_accel - max_accel_change,
                        min(last_accel + max_accel_change, requested_accel))
            velocity = prior + accel * dt

            # [CHANGED] Do not let a transient positive acceleration continue
            # after release, or a braking trajectory cross the zero boundary.
            # A hard brake-to-zero may override the jerk bound for safety.
            if target >= prior:
                velocity = max(prior, min(target, velocity))
            else:
                velocity = min(prior, max(target, velocity))
            if target == 0.0 and prior != 0.0 and velocity * prior < 0.0:
                velocity = 0.0
            if abs(velocity) < self.STOP_EPS and abs(target) < self.STOP_EPS:
                velocity = 0.0

            velocity = max(-self.SPEED[index], min(self.SPEED[index], velocity))
            self._acceleration[index] = (velocity - prior) / dt
            next_output.append(velocity)
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


def lift_target(value, previous, measured, limits, elapsed, speed=.12):
    """Right-stick lift independent of yaw; same sign, continuous diagonal input.

    The prior >.95 edge test rejects a circular stick diagonal (~.707).
    Integrate in metres/second instead of increments per WebXR event.
    """
    if not math.isfinite(elapsed) or elapsed <= 0 or not math.isfinite(speed) or speed <= 0:
        raise ValueError('invalid lift timing/speed')
    if not math.isfinite(value) or abs(value) > 1.05:
        raise ValueError('invalid lift axis')
    bound = limits['lift_joint']
    lower,upper,velocity=(float(bound[k]) for k in ('lower','upper','velocity'))
    base=float(measured if previous is None else previous)
    if not all(map(math.isfinite,(lower,upper,velocity,base))) or lower>=upper or velocity<=0:
        raise ValueError('invalid lift limit/feedback')
    demand=_axis(value,.10)
    return max(lower,min(upper,base+demand*min(speed,velocity)*min(elapsed,.05)))


class BodyMotionPolicy:
    """Keep elbows relaxed through commanded/observed chassis movement and settling."""
    def __init__(self):
        self.previous_lift = None
        self.last_motion = -math.inf
        self.active = False

    def update(self, now, base, lift, lift_target_value):
        values = (*base,lift,now)
        if len(base)!=3 or not all(math.isfinite(float(v)) for v in values):
            raise ValueError('invalid body motion observation')
        moving = any(abs(v)>1e-5 for v in base)
        if self.previous_lift is not None:
            moving = moving or abs(lift-self.previous_lift)>.00005
        if lift_target_value is not None:
            if not math.isfinite(lift_target_value):raise ValueError('invalid lift target')
            moving = moving or abs(lift_target_value-lift)>.002
        self.previous_lift = lift
        if moving:self.last_motion = now
        self.active = moving or now-self.last_motion<.25
        return self.active

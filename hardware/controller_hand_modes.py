"""Controller preset selection; no ROS or hardware side effects."""
import time

POSE_BUTTONS = {'X':'full_grip','Y':'angled_grip','A':'tripod_deep','B':'ulnar_pinch'}  # [CHANGED] A/B select new pinch targets
POSE_CODES = {'full_grip':1,'angled_grip':2,'tripod_deep':3,'ulnar_pinch':4}  # [CHANGED] mode 2 codes
BUTTON_RELEASE_GRACE = .15


class ControllerHandModes:
    def __init__(self):
        self.selected = 'full_grip'
        self.previous = frozenset()
        self.candidate = None
        self.chord = False
        self.pending = None
        self.blocked_until = -float('inf')
        self.captured = {'left':None,'right':None}

    def buttons(self,left,right,now=None):
        now = time.monotonic() if now is None else now
        pressed = frozenset(key for key,down in (
            ('X',left.get('aButton',False)),('Y',left.get('bButton',False)),
            ('A',right.get('aButton',False)),('B',right.get('bButton',False))) if down)
        changed = None
        if (len(pressed)>1 or (pressed and now<self.blocked_until)
                or (pressed and self.pending is not None)
                or (pressed and self.previous and pressed!=self.previous)):
            self.chord = True
            self.candidate = None
            self.pending = None
        elif pressed and not self.previous and not self.chord:
            self.candidate = next(iter(pressed))
        elif not pressed:
            # Wait for a quiet release interval before committing a tap.
            # Cross-controller event skew must not turn AX/BY into a preset.
            if self.chord:self.blocked_until=now+BUTTON_RELEASE_GRACE
            if self.candidate is not None and not self.chord:
                self.pending = (self.candidate,now)
            self.candidate = None
            self.chord = False
            if self.pending is not None and now-self.pending[1]>=BUTTON_RELEASE_GRACE:
                button,_ = self.pending
                self.selected = POSE_BUTTONS[button]
                changed = (button,self.selected)
                self.pending = None
        self.previous = pressed
        return changed

    def reset_grasps(self):
        self.captured = dict.fromkeys(self.captured)

    def reset_buttons(self):
        self.previous = frozenset()
        self.candidate = None
        self.pending = None
        self.chord = True  # require an observed all-released frame

    def controls(self,values):
        result = {}
        for side,item in values.items():
            if item['trigger']<=.02:self.captured[side] = None
            elif item['trigger']>.05 and self.captured[side] is None:
                self.captured[side] = self.selected
            result[side] = dict(item,pose=self.captured[side] or self.selected)
        return result

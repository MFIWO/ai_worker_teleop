"""Controller preset selection; no ROS or hardware side effects."""
POSE_BUTTONS = {'X':'full_grip','Y':'angled_grip','B':'point','A':'thumb_out_grip'}
POSE_CODES = {'full_grip':1,'angled_grip':2,'point':3,'thumb_out_grip':4}


class ControllerHandModes:
    def __init__(self):
        self.selected = 'full_grip'
        self.previous = frozenset()
        self.candidate = None
        self.chord = False
        self.captured = {'left':None,'right':None}

    def buttons(self,left,right):
        pressed = frozenset(key for key,down in (
            ('X',left.get('aButton',False)),('Y',left.get('bButton',False)),
            ('A',right.get('aButton',False)),('B',right.get('bButton',False))) if down)
        changed = None
        if len(pressed)>1 or (pressed and self.previous and pressed!=self.previous):
            self.chord = True
            self.candidate = None
        elif pressed and not self.previous and not self.chord:
            self.candidate = next(iter(pressed))
        elif not pressed:
            # Commit a single-button tap on release. AX/BY and their partial
            # releases never select a hand pose or move a finger.
            if self.candidate is not None and not self.chord:
                self.selected = POSE_BUTTONS[self.candidate]
                changed = (self.candidate,self.selected)
            self.candidate = None
            self.chord = False
        self.previous = pressed
        return changed

    def reset_grasps(self):
        self.captured = dict.fromkeys(self.captured)

    def reset_buttons(self):
        self.previous = frozenset()
        self.candidate = None
        self.chord = True  # require an observed all-released frame

    def controls(self,values):
        result = {}
        for side,item in values.items():
            if item['trigger']<=.02:self.captured[side] = None
            elif item['trigger']>.05 and self.captured[side] is None:
                self.captured[side] = self.selected
            result[side] = dict(item,pose=self.captured[side] or self.selected)
        return result

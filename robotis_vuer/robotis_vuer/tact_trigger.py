"""AI Worker 리더 손잡이 버튼(tact switch) 동작을 VR/XR 컨트롤러 버튼으로 흉내 낸다.

ffw_joystick_controller/src/joystick_controller.cpp 의 handle_tact_switches() 와 같은 규칙:
  - 버튼은 '뗄 때' 판단한다 (모든 버튼이 떨어진 순간).
  - 한쪽만 눌렀다 떼면       → 'left' / 'right'
  - 한쪽을 long_press 초 이상 누르면 (누르고 있는 중에 1회) → 'left_long_time' / 'right_long_time'
    이 경우 뗄 때 'left' / 'right' 는 보내지 않는다.
  - 양쪽을 함께 누른 적이 있으면 뗄 때 'both' (리더는 모드 전환) 만, 단독 신호는 보내지 않는다.

Cyclo Intelligence 는 /leader/joystick_controller/tact_trigger (std_msgs/String) 의
'left' = 에피소드 취소, 'right' = 녹화 시작/저장 으로 사용한다.
"""

TACT_TRIGGER_TOPIC = '/leader/joystick_controller/tact_trigger'


class TactTrigger:
    def __init__(self, long_press_duration=2.0):
        self.long_press_duration = float(long_press_duration)
        self.prev_left = False
        self.prev_right = False
        self.left_start = None
        self.right_start = None
        self.left_long = False
        self.right_long = False
        self.both_pressed = False

    def update(self, left_pressed, right_pressed, now):
        """버튼 상태와 현재 시각(초)을 넣으면 이번에 보낼 이벤트 목록을 돌려준다."""
        left_pressed, right_pressed = bool(left_pressed), bool(right_pressed)
        events = []
        current = (2 if left_pressed else 0) | (1 if right_pressed else 0)
        prev = (2 if self.prev_left else 0) | (1 if self.prev_right else 0)

        if left_pressed and not self.prev_left:
            self.left_start, self.left_long = now, False
        if right_pressed and not self.prev_right:
            self.right_start, self.right_long = now, False

        if left_pressed and not self.left_long and now - self.left_start >= self.long_press_duration:
            events.append('left_long_time')
            self.left_long = True
        if right_pressed and not self.right_long and now - self.right_start >= self.long_press_duration:
            events.append('right_long_time')
            self.right_long = True

        if current == 3:
            self.both_pressed = True

        if current == 0 and prev != 0:
            if self.both_pressed:
                events.append('both')
                self.both_pressed = False
            elif prev == 1 and not self.right_long:
                events.append('right')
            elif prev == 2 and not self.left_long:
                events.append('left')
            if prev in (1, 3):
                self.right_long = False
            if prev in (2, 3):
                self.left_long = False

        self.prev_left, self.prev_right = left_pressed, right_pressed
        return events

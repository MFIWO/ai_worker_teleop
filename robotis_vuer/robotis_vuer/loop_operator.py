"""Loop-focused X11 keyboard bridge. Publishes input intent, never motor topics."""
import ctypes as C
import json
import signal
import time
import uuid

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String

from hardware.keyboard_controls import MOTION_KEYS
from robotis_vuer.loop_keys import X11Keys, XEvent


class LoopKeyboard(X11Keys):
    def __init__(self):
        super().__init__()
        self.x.XQueryKeymap.argtypes=[C.c_void_p,C.POINTER(C.c_char)]
        self.x.XQueryKeymap.restype=C.c_int
        self.x.XGrabKey.argtypes=[C.c_void_p,C.c_int,C.c_uint,C.c_ulong,C.c_int,C.c_int,C.c_int]
        self.x.XUngrabKey.argtypes=[C.c_void_p,C.c_int,C.c_uint,C.c_ulong]
        self.x.XPending.argtypes=[C.c_void_p]
        self.x.XNextEvent.argtypes=[C.c_void_p,C.POINTER(XEvent)]
        names={key:key.capitalize() if key in ('up','down','left','right') else key for key in MOTION_KEYS|{'c','v'}}
        self.codes={key:self.x.XKeysymToKeycode(self.display,self.x.XStringToKeysym(name.encode()))
                    for key,name in names.items()}
        self.modifiers=[self.x.XKeysymToKeycode(self.display,self.x.XStringToKeysym(name.encode()))
                        for name in ('Shift_L','Shift_R','Control_L','Control_R','Alt_L','Alt_R','Super_L','Super_R')]
        self.window=0
        self.focused=False
        self.armed=False
        self.c_down=False
        self.v_down=False

    def release_grabs(self):
        if self.window:
            for code in self.codes.values():
                self.x.XUngrabKey(self.display,code,1<<15,self.window)  # AnyModifier
            self.x.XSync(self.display,False)
        self.window=0

    def poll(self):
        try:window=self.loop_window()
        except RuntimeError:window=0
        if window!=self.window:
            self.release_grabs()
            self.errors.clear()
            if window:
                for code in self.codes.values():
                    for modifiers in (0,2,16,18):  # Caps/Num Lock do not alter the mapping.
                        self.x.XGrabKey(self.display,code,modifiers,window,False,1,1)
                self.x.XSync(self.display,False)
                if self.errors:raise RuntimeError('Cannot grab Loop motion keys; another operator may be running')
            self.window=window
        # Drain grabbed key events; XQueryKeymap gives held state without auto-repeat.
        while self.x.XPending(self.display):
            event=XEvent()
            self.x.XNextEvent(self.display,C.byref(event))
        bitmap=C.create_string_buffer(32)
        self.x.XQueryKeymap(self.display,bitmap)
        bits=bitmap.raw
        down=lambda code:bool(code and bits[code//8]&(1<<(code%8)))
        focused=bool(self.window and self.active_window()==self.window)
        keys={key for key,code in self.codes.items() if down(code)} if focused else set()
        modified=any(down(code) for code in self.modifiers)
        toggle='c' in keys and not self.c_down and self.focused and not modified
        self.c_down='c' in keys
        discard=focused and not modified and 'v' in keys
        stop=discard and not self.v_down
        if discard!=self.v_down:
            # Preserve Loop's native tap/hold review semantics. C itself is
            # grabbed above and never delivered to the recording UI.
            self.send(self.window if discard else self._discard_window,'c',discard)
            if discard:self._discard_window=self.window
            self.v_down=discard
        if not focused or not self.focused or stop or toggle or modified:
            self.armed=False
        self.focused=focused
        motions=keys&MOTION_KEYS
        if focused and not keys:self.armed=True
        return focused, sorted(motions) if self.armed else [], stop, toggle

    def close(self):
        try:
            if self.v_down:self.send(self._discard_window,'c',False)
        finally:
            self.release_grabs()
            super().close()


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    def interrupt(*_):raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,interrupt)
    signal.signal(signal.SIGINT,interrupt)
    node=Node('loop_vr_operator')
    publisher=node.create_publisher(String,'/xr_hardware/operator_keys',10)
    keyboard=None
    sequence=0
    instance=uuid.uuid4().hex
    last_focus=None
    def send(focused,keys,stop,toggle=False):
        nonlocal sequence
        sequence+=1
        publisher.publish(String(data=json.dumps(dict(instance=instance,sequence=sequence,
                            focused=focused,keys=keys,stop=stop,toggle=toggle))))
    try:
        keyboard=LoopKeyboard()
        print('[LOOP KEYS] Loop focus: WASD=base; arrows=head; U/I=lift; O/P=yaw; C=start/HOLD/resume. '
              'Hold V=Loop discard (tap V=save failed on review screen). '
              'Shift+A or UI button starts Recorder. Recording continues during HOLD.',flush=True)
        while rclpy.ok():
            event=keyboard.poll()
            if event[0]!=last_focus:
                print('[LOOP FOCUS] '+('active; release held keys before motion' if event[0] else 'inactive; motion keys require the Loop window'),flush=True)
                last_focus=event[0]
            send(*event)
            rclpy.spin_once(node,timeout_sec=0.)
            time.sleep(.03)
    except KeyboardInterrupt:
        pass
    finally:
        send(False,[],True)
        rclpy.spin_once(node,timeout_sec=.05)
        if keyboard is not None:keyboard.close()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()

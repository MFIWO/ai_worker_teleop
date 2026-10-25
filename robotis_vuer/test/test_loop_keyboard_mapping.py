"""Exercise real keyboard polling with a fake X11 display; no ROS or robot."""
import ast
import ctypes as C
from pathlib import Path
import unittest
from unittest.mock import Mock


class LoopKeyboardMappingTests(unittest.TestCase):
    def setUp(self):
        source=Path(__file__).resolve().parents[1]/'robotis_vuer/loop_operator.py'
        cls=next(n for n in ast.parse(source.read_text()).body if isinstance(n,ast.ClassDef))
        scope={'C':C,'X11Keys':object,'XEvent':C.c_int,'MOTION_KEYS':{'w'}}
        exec(compile(ast.fix_missing_locations(ast.Module(body=[cls],type_ignores=[])),str(source),'exec'),scope)
        self.keyboard=scope['LoopKeyboard'].__new__(scope['LoopKeyboard'])
        k=self.keyboard
        k.display=1
        k.window=42
        k.focused=True
        k.armed=True
        k.toggle_down=k.discard_down=False
        k.codes={'c':10,'v':11,'w':12}
        k.modifiers=[13]
        k.loop_window=Mock(return_value=42)
        k.active_window=Mock(return_value=42)
        k.send=Mock()
        self.keys=set()
        def query(display,buffer):
            bitmap=bytearray(32)
            for code in self.keys:bitmap[code//8]|=1<<(code%8)
            buffer.raw=bytes(bitmap)
        k.x=Mock()
        k.x.XPending.return_value=0
        k.x.XQueryKeymap.side_effect=query

    def poll(self,*codes):
        self.keys=set(codes)
        return self.keyboard.poll()

    def test_v_toggles_once_without_forwarding_to_loop(self):
        self.assertEqual(self.poll(11),(True,[],False,True))
        self.assertEqual(self.poll(11),(True,[],False,False))
        self.poll()
        self.assertTrue(self.poll(11)[3])
        self.keyboard.send.assert_not_called()

    def test_c_preserves_native_press_hold_release_and_holds_teleop(self):
        self.assertEqual(self.poll(10),(True,[],True,False))
        self.assertEqual(self.poll(10),(True,[],False,False))
        self.poll()
        self.assertEqual(self.keyboard.send.call_args_list,
                         [((42,'c',True),),((42,'c',False),)])

    def test_focus_loss_releases_c_and_cannot_toggle_v(self):
        self.poll(10)
        self.keyboard.active_window.return_value=0
        self.assertEqual(self.poll(10,11),(False,[],False,False))
        self.keyboard.send.assert_called_with(42,'c',False)

    def test_modifiers_block_both_actions_and_toggle_clears_motion(self):
        self.assertEqual(self.poll(10,11,13),(True,[],False,False))
        self.keyboard.send.assert_not_called()
        self.poll()
        self.assertEqual(self.poll(11,12),(True,[],False,True))
        self.assertEqual(self.poll(12),(True,[],False,False))
        self.poll()
        self.assertEqual(self.poll(12),(True,['w'],False,False))


if __name__=='__main__':unittest.main()

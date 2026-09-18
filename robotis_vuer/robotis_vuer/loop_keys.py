"""X11 key delivery to Loop without activating its window (no extra dependency)."""

import ctypes as C
from ctypes.util import find_library


class XKeyEvent(C.Structure):
    _fields_ = [('type', C.c_int), ('serial', C.c_ulong), ('send_event', C.c_int),
                ('display', C.c_void_p), ('window', C.c_ulong), ('root', C.c_ulong),
                ('subwindow', C.c_ulong), ('time', C.c_ulong), ('x', C.c_int),
                ('y', C.c_int), ('x_root', C.c_int), ('y_root', C.c_int),
                ('state', C.c_uint), ('keycode', C.c_uint), ('same_screen', C.c_int)]


class XFocusEvent(C.Structure):
    _fields_ = [('type', C.c_int), ('serial', C.c_ulong), ('send_event', C.c_int),
                ('display', C.c_void_p), ('window', C.c_ulong),
                ('mode', C.c_int), ('detail', C.c_int)]


class XEvent(C.Union):
    _fields_ = [('key', XKeyEvent), ('focus', XFocusEvent), ('pad', C.c_long * 24)]


class X11Keys:
    def __init__(self):
        self.x = C.CDLL(find_library('X11'))
        signatures = {
            'XOpenDisplay': ([C.c_char_p], C.c_void_p),
            'XDefaultRootWindow': ([C.c_void_p], C.c_ulong),
            'XInternAtom': ([C.c_void_p, C.c_char_p, C.c_int], C.c_ulong),
            'XGetWindowProperty': ([C.c_void_p, C.c_ulong, C.c_ulong, C.c_long,
                                   C.c_long, C.c_int, C.c_ulong, C.POINTER(C.c_ulong),
                                   C.POINTER(C.c_int), C.POINTER(C.c_ulong),
                                   C.POINTER(C.c_ulong), C.POINTER(C.c_void_p)], C.c_int),
            'XStringToKeysym': ([C.c_char_p], C.c_ulong),
            'XKeysymToKeycode': ([C.c_void_p, C.c_ulong], C.c_uint),
            'XSendEvent': ([C.c_void_p, C.c_ulong, C.c_int, C.c_long,
                            C.POINTER(XEvent)], C.c_int),
            'XFlush': ([C.c_void_p], C.c_int),
            'XSync': ([C.c_void_p, C.c_int], C.c_int),
            'XFree': ([C.c_void_p], C.c_int),
            'XCloseDisplay': ([C.c_void_p], C.c_int),
        }
        for name, (args, result) in signatures.items():
            fn = getattr(self.x, name)
            fn.argtypes, fn.restype = args, result
        self.display = self.x.XOpenDisplay(None)
        if not self.display:
            raise RuntimeError(
                'X11 display unavailable; run on the local PC with DISPLAY/XAUTHORITY')
        self.root = self.x.XDefaultRootWindow(self.display)
        # A window may close between discovery and delivery; report errors without aborting Python.
        self.errors = []
        self._error_callback = C.CFUNCTYPE(C.c_int, C.c_void_p, C.c_void_p)(
            lambda display, event: self.errors.append('X11 window disappeared') or 0)
        self.x.XSetErrorHandler.argtypes = [C.c_void_p]
        self.x.XSetErrorHandler.restype = C.c_void_p
        self._previous_error_handler = self.x.XSetErrorHandler(self._error_callback)
        self._held = set()

    def get_property(self, window, name):
        atom = self.x.XInternAtom(self.display, name.encode(), False)
        actual, fmt, count, remaining, data = (
            C.c_ulong(), C.c_int(), C.c_ulong(), C.c_ulong(), C.c_void_p())
        code = self.x.XGetWindowProperty(
            self.display, window, atom, 0, 4096, False, 0,
            C.byref(actual), C.byref(fmt), C.byref(count), C.byref(remaining), C.byref(data))
        if code or not data.value:
            return [] if fmt.value == 32 else b''
        try:
            if fmt.value == 32:
                return list(C.cast(data, C.POINTER(C.c_ulong))[:count.value])
            return C.string_at(data, count.value * max(1, fmt.value // 8))
        finally:
            self.x.XFree(data)

    def active_window(self):
        windows = self.get_property(self.root, '_NET_ACTIVE_WINDOW')
        return windows[0] if windows else 0

    def loop_window(self):
        found = []
        for window in self.get_property(self.root, '_NET_CLIENT_LIST'):
            cls = self.get_property(window, 'WM_CLASS').lower().split(b'\0')
            if b'loop' in cls:
                found.append(window)
        if len(found) != 1:
            raise RuntimeError(f'Expected one Loop window, found {len(found)}; open Loop Recorder')
        return found[0]

    def focus_event(self, window, focused):
        event = XEvent()
        event.focus.type = 9 if focused else 10
        event.focus.display = self.display
        event.focus.window = window
        event.focus.mode = 0
        event.focus.detail = 3
        self.x.XSendEvent(self.display, window, False, 1 << 21, C.byref(event))
        self.x.XSync(self.display, False)

    def send(self, window, key, pressed):
        if key not in ('a', 'b', 'c'):
            raise ValueError('Only Loop A/B/C may be forwarded')
        self.errors.clear()
        if pressed and not any(w == window for w, _ in self._held):
            self.focus_event(window, True)
        event = XEvent()
        event.key.type = 2 if pressed else 3
        event.key.display = self.display
        event.key.window = window
        event.key.root = self.root
        event.key.same_screen = True
        event.key.keycode = self.x.XKeysymToKeycode(
            self.display, self.x.XStringToKeysym(key.encode()))
        ok = self.x.XSendEvent(self.display, window, True, 1 if pressed else 2,
                               C.byref(event))
        self.x.XSync(self.display, False)
        if not ok or self.errors:
            raise RuntimeError('Loop key delivery failed')
        if pressed:
            self._held.add((window, key))
        else:
            self._held.discard((window, key))
            if not any(w == window for w, _ in self._held) and self.active_window() != window:
                self.focus_event(window, False)

    def close(self):
        self.x.XCloseDisplay(self.display)
        self.x.XSetErrorHandler(self._previous_error_handler)

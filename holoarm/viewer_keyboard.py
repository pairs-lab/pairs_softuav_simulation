"""Window-local X11 key grabs and physical held-key polling.

MuJoCo's passive key callback has no release events and does not consume its
native debug shortcuts. Grab only our keys on our viewer window, not globally,
so they never reach those shortcuts. Mouse/UI and other apps stay untouched.
"""
import ctypes as C
import ctypes.util
import time


KEY_NAMES = ('i', 'k', 'j', 'l', 'u', 'o', 'q', 'e', 'h', 'r', 'f',
             'Up', 'Down', 'Left', 'Right', 'space', 'Escape')
KEY_ALIASES = {'Up': 'I', 'Down': 'K', 'Left': 'J', 'Right': 'L',
               'space': 'SPACE', 'Escape': 'ESC'}


class X11Keyboard:
    def __init__(self, window_title):
        library = ctypes.util.find_library('X11')
        if not library:
            raise RuntimeError('Continuous keyboard flight requires libX11 on this desktop')
        self.x = C.CDLL(library)
        display, window = C.c_void_p, C.c_ulong
        signatures = {
            'XOpenDisplay': ([C.c_char_p], display),
            'XCloseDisplay': ([display], C.c_int),
            'XDefaultRootWindow': ([display], window),
            'XQueryTree': ([display, window, C.POINTER(window), C.POINTER(window),
                           C.POINTER(C.POINTER(window)), C.POINTER(C.c_uint)], C.c_int),
            'XFetchName': ([display, window, C.POINTER(C.c_void_p)], C.c_int),
            'XFree': ([C.c_void_p], C.c_int),
            'XStringToKeysym': ([C.c_char_p], C.c_ulong),
            'XKeysymToKeycode': ([display, C.c_ulong], C.c_ubyte),
            'XGrabKey': ([display, C.c_int, C.c_uint, window, C.c_int, C.c_int, C.c_int], C.c_int),
            'XUngrabKey': ([display, C.c_int, C.c_uint, window], C.c_int),
            'XQueryKeymap': ([display, C.c_void_p], C.c_int),
            'XGetInputFocus': ([display, C.POINTER(window), C.POINTER(C.c_int)], C.c_int),
            'XPending': ([display], C.c_int),
            'XNextEvent': ([display, C.c_void_p], C.c_int),
            'XSync': ([display, C.c_int], C.c_int),
        }
        for name, (args, result) in signatures.items():
            function = getattr(self.x, name)
            function.argtypes, function.restype = args, result
        self.display = self.x.XOpenDisplay(None)
        self.window = None
        if not self.display:
            raise RuntimeError('Cannot connect to the desktop X11 display')
        try:
            deadline = time.monotonic()+3
            while self.window is None:
                self.window = self._find_window(self.x.XDefaultRootWindow(self.display), window_title)
                if time.monotonic() > deadline:
                    raise RuntimeError('Cannot locate the MuJoCo keyboard-flight window')
                if self.window is None:
                    time.sleep(.02)
            self.codes = {}
            for name in KEY_NAMES:
                symbol = self.x.XStringToKeysym(name.encode())
                code = int(self.x.XKeysymToKeycode(self.display, symbol))
                self.codes[name] = code
                # AnyModifier, owner_events=False, both modes asynchronous.
                self.x.XGrabKey(self.display, code, 1 << 15, self.window, 0, 1, 1)
            self.x.XSync(self.display, 0)
        except BaseException:
            self.close()
            raise

    def _tree(self, win):
        root, parent = C.c_ulong(), C.c_ulong()
        children, count = C.POINTER(C.c_ulong)(), C.c_uint()
        if not self.x.XQueryTree(self.display, win, C.byref(root), C.byref(parent),
                                 C.byref(children), C.byref(count)):
            return 0, []
        ids = [int(children[i]) for i in range(count.value)]
        if children:
            self.x.XFree(children)
        return parent.value, ids

    def _find_window(self, win, title):
        name = C.c_void_p()
        if self.x.XFetchName(self.display, win, C.byref(name)) and name.value:
            text = C.string_at(name).decode(errors='replace')
            self.x.XFree(name)
            if text == title:
                # Window-manager frames have the same name. Prefer the
                # deepest matching client window, where GLFW receives keys.
                for child in self._tree(win)[1]:
                    found = self._find_window(child, title)
                    if found:
                        return found
                return win
        for child in self._tree(win)[1]:
            found = self._find_window(child, title)
            if found:
                return found
        return None

    def poll(self):
        # Drain grabbed events, including repeat pairs; actual state comes
        # from QueryKeymap so repeat never creates false release/press edges.
        event = (C.c_long*24)()
        while self.x.XPending(self.display):
            self.x.XNextEvent(self.display, C.byref(event))
        focus, revert = C.c_ulong(), C.c_int()
        self.x.XGetInputFocus(self.display, C.byref(focus), C.byref(revert))
        focused = focus.value == self.window
        ancestor = focus.value
        while not focused and ancestor not in (0, 1):
            ancestor = self._tree(ancestor)[0]
            focused = ancestor == self.window
        if not focused:
            return set()
        state = (C.c_ubyte*32)()
        self.x.XQueryKeymap(self.display, state)
        return {KEY_ALIASES.get(name, name.upper()) for name, code in self.codes.items()
                if state[code//8] & (1 << (code % 8))}

    def close(self):
        if self.display:
            # Closing the connection releases all grabs, even if the viewer
            # has already destroyed its window.
            self.x.XCloseDisplay(self.display)
            self.display = None

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()

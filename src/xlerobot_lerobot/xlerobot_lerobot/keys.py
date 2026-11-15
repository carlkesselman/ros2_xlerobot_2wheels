"""Single key presses from the terminal, without Enter, for the recorder and the policy node.

lerobot's own listener uses pynput, which on macOS needs the terminal to have Accessibility access and reads keys
from every window. This reads only the terminal the tool runs in.
"""
import os
import select
import sys
import termios
import threading
import tty

NAMES = {"\x1b[C": "right", "\x1b[D": "left", "\x1b[A": "up", "\x1b[B": "down", "\x1b": "esc",
         "\r": "enter", "\n": "enter", " ": "space"}


class KeyReader:
    """Calls on_key(name) from a background thread for each key: arrows as left/right/up/down, esc, enter, space,
    anything else as the character. Restores the terminal on exit. Does nothing when stdin is not a terminal."""

    def __init__(self, on_key):
        self.on_key = on_key
        self.fd = sys.stdin.fileno() if sys.stdin.isatty() else None
        self.saved = None
        self.stop = threading.Event()
        self.thread = None

    def __enter__(self):
        if self.fd is None:
            return self
        self.saved = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()
        return self

    def __exit__(self, *exc):
        self.stop.set()
        if self.saved is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.saved)
        return False

    def _read(self, timeout):
        r, _, _ = select.select([self.fd], [], [], timeout)
        return os.read(self.fd, 1).decode(errors="ignore") if r else ""

    def _run(self):
        while not self.stop.is_set():
            ch = self._read(0.2)
            if not ch:
                continue
            if ch == "\x1b":               # an arrow key is ESC [ X; a lone ESC is the key itself
                nxt = self._read(0.05)
                if nxt == "[":
                    ch += nxt + self._read(0.05)
                elif nxt:
                    ch = nxt
            self.on_key(NAMES.get(ch, ch))

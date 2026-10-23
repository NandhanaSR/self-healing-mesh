"""Keyboard input for the dashboard: msvcrt on Windows, termios on POSIX.

Keys:  Tab / ]  select next board      [  select previous board
       0-5      send that digit to the selected board (its data destination)
       p        send 'p' to the selected board (toggle priority)
       space    pause / resume the event log (events are still recorded while paused)
       f        event log filter: all / important only (hides INFO)
       q        quit (Ctrl+C also quits)
Anything else, including arrow and function keys, is ignored.
"""
from __future__ import annotations

import copy
import os
import sys

from live_model import MAX_NODES

SEND_KEYS = {str(i) for i in range(MAX_NODES)} | {"p"}      # what mesh_node accepts on serial
QUIT_KEYS = {"q", "\x03"}                                    # \x03 = Ctrl+C read as a character
NEXT_KEYS = {"\t", "]"}
PREV_KEYS = {"["}
PAUSE_KEY = " "
FILTER_KEY = "f"


class KeyControl:
    """Turns keys into actions. Keeps the selected board (by port, in --ports order)."""

    def __init__(self, mesh):
        self.mesh = mesh
        self.ports = list(mesh.boards)
        self.index = 0
        self.paused = None                  # event log frozen: copies of its rows at pause time
        self.paused_seq = 0
        self.important_only = False         # event log filter: hide INFO rows

    @property
    def selected(self):
        return self.ports[self.index] if self.ports else None

    def target_label(self) -> str:
        b = self.mesh.boards.get(self.selected)
        if b is None:
            return "—"
        return f"N{b.node_id} ({b.port})" if b.node_id is not None else b.port

    def handle(self, key: str, now: float) -> bool:
        """Act on one key. Returns False when the dashboard should quit."""
        if key in QUIT_KEYS:
            return False
        if not self.ports:
            return True
        if key in NEXT_KEYS:
            self.index = (self.index + 1) % len(self.ports)
        elif key in PREV_KEYS:
            self.index = (self.index - 1) % len(self.ports)
        elif key in SEND_KEYS:
            if self.mesh.send(self.selected, key):
                self.mesh.log(now, "INFO", self.target_label(), f"sent '{key}'")
            else:
                self.mesh.log(now, "WARN", self.selected, f"not open, key '{key}' ignored")
        elif key == PAUSE_KEY:
            if self.paused is None:
                self.paused = [copy.copy(e) for e in self.mesh.events]   # rows as they are now
                self.paused_seq = self.mesh.event_seq
            else:
                self.paused = None
        elif key == FILTER_KEY:
            self.important_only = not self.important_only
        return True

    def new_since_pause(self) -> int:
        """New or updated event rows since the log was paused."""
        return sum(1 for e in self.mesh.events if e.seq > self.paused_seq)


class Keyboard:
    """Non-blocking key reader. Use as a context manager; read() returns the keys pressed since
    the last call. Disabled (read() returns []) when stdin is not a terminal."""

    def __init__(self, stream=None):
        self.stream = stream or sys.stdin
        self.enabled = False
        self._saved = None

    def __enter__(self):
        try:
            if not self.stream.isatty():
                return self
        except (AttributeError, ValueError):
            return self
        if os.name == "nt":
            self.enabled = True
        else:
            import termios
            import tty
            fd = self.stream.fileno()
            self._saved = termios.tcgetattr(fd)
            tty.setcbreak(fd)               # keys arrive one at a time, no echo; Ctrl+C still signals
            self.enabled = True
        return self

    def __exit__(self, *exc):
        if self._saved is not None:
            import termios
            termios.tcsetattr(self.stream.fileno(), termios.TCSADRAIN, self._saved)
            self._saved = None
        self.enabled = False
        return False

    def read(self) -> list:
        if not self.enabled:
            return []
        return self._read_windows() if os.name == "nt" else self._read_posix()

    @staticmethod
    def _read_windows() -> list:
        import msvcrt
        keys = []
        while msvcrt.kbhit():
            ch = msvcrt.getwch()
            if ch in ("\x00", "\xe0"):      # arrow / function key: a second code follows, drop both
                msvcrt.getwch()
                continue
            keys.append(ch)
        return keys

    def _read_posix(self) -> list:
        import select
        fd = self.stream.fileno()
        data = ""
        while select.select([fd], [], [], 0)[0]:
            chunk = os.read(fd, 64)
            if not chunk:
                break
            data += chunk.decode("utf-8", errors="ignore")
        return split_posix_keys(data)


def split_posix_keys(data: str) -> list:
    """Split raw terminal input into keys, dropping escape sequences (arrows, F-keys: ESC [ ... X)."""
    keys, i = [], 0
    while i < len(data):
        ch = data[i]
        i += 1
        if ch != "\x1b":
            keys.append(ch)
            continue
        if i < len(data) and data[i] in "[O":
            i += 1
            while i < len(data) and not (data[i].isalpha() or data[i] == "~"):
                i += 1
            i += 1                          # the final byte of the sequence
    return keys

"""DEMO sensor data for the anomaly panel. GENERATED, not from the boards.

No sensors are connected (pins 34/35 float), so the s1/s2/anomaly fields in [DATA] lines
are noise and are ignored. Instead this module generates two synthetic sensor traces per
board, injects spikes / jumps / stuck stretches, and runs a Python copy of the firmware's
detector (anomaly.cpp) on them so the panel shows what the module would flag.
"""
from __future__ import annotations

import math
import random
import statistics
from collections import deque

# config.h / packets.h
AN_WINDOW, AN_MIN_SAMPLES = 10, 5
AN_SPIKE_SIGMA, AN_MIN_STD = 3.0, 10.0
AN_JUMP_DELTA = 800
AN_STUCK_TOL, AN_STUCK_COUNT = 2, 20
ANOM_SPIKE, ANOM_STUCK, ANOM_JUMP = 0x01, 0x02, 0x04
FLAG_NAMES = ((ANOM_SPIKE, "SPIKE"), (ANOM_JUMP, "JUMP"), (ANOM_STUCK, "STUCK"))

HIST_LEN = 16
SHOW_FLAG_S = 5             # keep a flag on screen for this many samples


def flag_names(flags):
    return [name for bit, name in FLAG_NAMES if flags & bit]


class Detector:
    """anomaly_update() from anomaly.cpp."""

    def __init__(self):
        self.buf = deque(maxlen=AN_WINDOW)
        self.prev = None
        self.same = 0

    def update(self, x):
        flags = 0
        if len(self.buf) >= AN_MIN_SAMPLES:
            mean = sum(self.buf) / len(self.buf)
            sd = max(statistics.pstdev(self.buf), AN_MIN_STD)
            if abs(x - mean) > AN_SPIKE_SIGMA * sd:
                flags |= ANOM_SPIKE
        if self.prev is not None:
            d = abs(x - self.prev)
            if d > AN_JUMP_DELTA:
                flags |= ANOM_JUMP
            self.same = min(self.same + 1, 255) if d <= AN_STUCK_TOL else 0
            if self.same >= AN_STUCK_COUNT:
                flags |= ANOM_STUCK
        self.buf.append(x)
        self.prev = x
        return flags


class DemoSensor:
    def __init__(self, rng):
        self.rng = rng
        self.phase = rng.uniform(0, 6.28)
        self.offset = 0                 # jump: shifted baseline
        self.offset_left = 0
        self.stuck_left = 0
        self.value = 2048
        self.det = Detector()
        self.hist = deque(maxlen=HIST_LEN)
        self.last_flags, self.flag_age = 0, SHOW_FLAG_S
        self.counts = {name: 0 for _, name in FLAG_NAMES}

    def step(self, t):
        r = self.rng.random()
        if self.stuck_left:
            self.stuck_left -= 1
            x = self.value                                      # frozen reading
        else:
            if self.offset_left:
                self.offset_left -= 1
                if not self.offset_left:
                    self.offset = 0
            elif r < 0.02:
                self.offset, self.offset_left = self.rng.choice((-1100, 1100)), self.rng.randint(6, 12)
            elif r < 0.03:
                self.stuck_left = AN_STUCK_COUNT + 5
            x = 2048 + 500 * math.sin(2 * math.pi * t / 30.0 + self.phase) + self.rng.gauss(0, 25) + self.offset
            if self.rng.random() < 0.03:
                x += self.rng.choice((-1, 1)) * 1500              # one-sample spike
            x = int(max(0, min(4095, x)))
        self.value = x
        flags = self.det.update(x)
        self.hist.append(x)
        if flags:
            self.last_flags, self.flag_age = flags, 0
            for name in flag_names(flags):
                self.counts[name] += 1
        else:
            self.flag_age += 1
        return flags

    @property
    def shown_flags(self):
        return self.last_flags if self.flag_age < SHOW_FLAG_S else 0


class DemoAnomalies:
    """Two generated sensors per node id; call step() once per second."""

    def __init__(self, seed=None):
        self.rng = random.Random(seed)
        self.t = 0
        self.sensors = {}               # node -> (s1, s2)

    def step(self, nodes):
        self.t += 1
        for n in nodes:
            pair = self.sensors.setdefault(n, (DemoSensor(self.rng), DemoSensor(self.rng)))
            for s in pair:
                s.step(self.t)

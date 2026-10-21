"""Simulated boards for --demo. Everything they print is SIMULATED, never real board data.

Each SimBoard imitates one ESP32 running mesh_node.ino: it measures its links with the
firmware's formulas (link_health.cpp), picks next hop / priority next hop with the
firmware's rules (routing.cpp) and prints the firmware's exact serial lines. Each board
is exposed as a serial-port-like object, so the dashboard reads it through the same
SerialReader -> parser -> LiveMesh path as a real COM port.

Deliberate simplifications: radio is a logistic packet-reception model with scripted
fading, and routes are computed from a global view instead of exchanging adverts.
"""
from __future__ import annotations

import math
import random
import threading
import time
from collections import deque

# ---- config.h values the simulation follows
MAX_NODES = 6
DEFAULT_DEST_ID = 1
EWMA_ALPHA = 0.30
RSSI_MIN, RSSI_MAX = -85, -40
SLOPE_WINDOW = 5
W_RSSI, W_DELIVERY = 0.6, 0.4
PRR_WINDOW_TX = 5
PREDICT_HORIZON_S = 3.0
Q_TH, Q_RECOVER = 0.50, 0.65
DEAD_TIMEOUT_S = 2.5
HEARTBEAT_S = 0.5
PRINT_S = 1.0               # PRINT_INTERVAL_MS and DATA_INTERVAL_MS
COST_HEALTHY, COST_SCALE = 1.0, 10.0
ROUTE_HYST_MARGIN = 0.10
MAX_TTL = 6
INF = float("inf")

# ---- simulated radio
NODES = (0, 1, 2, 3)
LINKS = [(0, 2, -60), (1, 2, -58), (0, 3, -66), (1, 3, -68), (2, 3, -63)]   # (a, b, base RSSI dBm)
CYCLE = 60.0                # seconds in the fading script
SENS, K = -88.0, 2.5        # reception probability 50% at SENS dBm, logistic slope K


def trap(x, a, b, c, d, depth):
    """Trapezoid: ramps 0->depth over [a,b], holds to c, ramps back by d."""
    if x <= a or x >= d:
        return 0.0
    if x < b:
        return depth * (x - a) / (b - a)
    if x <= c:
        return depth
    return depth * (d - x) / (d - c)


def fade(a, b, t):
    tc = t % CYCLE
    key = (min(a, b), max(a, b))
    if key == (1, 2):
        return trap(tc, 8, 18, 30, 38, -40)     # N1-N2 degrades, dies, comes back
    if key == (1, 3):
        return trap(tc, 40, 46, 52, 56, -16)    # N1-N3 degrades for a while
    return 0.0


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


# --------------------------------------------------------------------------- link_health.cpp


class SimLink:
    def __init__(self, rssi, now):
        self.reset(rssi, now)

    def reset(self, rssi, now):
        self.ewma, self.slope, self.prr, self.dprr = float(rssi), 0.0, 1.0, 0.0
        self.last_seen = now
        self.hist = deque([(now, self.ewma)], maxlen=SLOPE_WINDOW)
        self.win_sent = self.win_acked = 0
        self.state = "HEALTHY"
        self._quality()

    def _quality(self):
        score = lambda r: (clamp(r, RSSI_MIN, RSSI_MAX) - RSSI_MIN) * 100.0 / (RSSI_MAX - RSSI_MIN)
        q = W_RSSI * score(self.ewma) / 100.0 + W_DELIVERY * self.prr
        self.q = int(clamp(q * 100.0, 0.0, 100.0))
        ahead = self.ewma + self.slope * PREDICT_HORIZON_S
        self.q_pred = clamp(W_RSSI * score(ahead) / 100.0 + W_DELIVERY * clamp(self.prr + self.dprr, 0.0, 1.0),
                            0.0, 1.0)

    def _slope(self):
        if len(self.hist) < 3:
            return 0.0
        t0 = self.hist[0][0]
        n = len(self.hist)
        st = sum(t - t0 for t, _ in self.hist)
        sy = sum(y for _, y in self.hist)
        stt = sum((t - t0) ** 2 for t, _ in self.hist)
        sty = sum((t - t0) * y for t, y in self.hist)
        den = n * stt - st * st
        return 0.0 if den < 1e-9 else (n * sty - st * sy) / den

    def update_state(self, now):
        if now - self.last_seen > DEAD_TIMEOUT_S:
            self.state = "DEAD"
        elif self.state == "HEALTHY" and self.q_pred < Q_TH:
            self.state = "DEGRADING"
        elif self.state == "DEGRADING" and self.q_pred > Q_RECOVER:
            self.state = "HEALTHY"

    def on_rx(self, rssi, now):
        if self.state == "DEAD":                  # back from the dead: probation
            self.reset(rssi, now)
            self.state = "DEGRADING"
            return
        self.ewma = EWMA_ALPHA * rssi + (1 - EWMA_ALPHA) * self.ewma
        self.last_seen = now
        self.hist.append((now, self.ewma))
        self.slope = self._slope()
        self._quality()
        self.update_state(now)

    def on_tx(self, ok, now):
        self.win_sent += 1
        self.win_acked += ok
        if self.win_sent >= PRR_WINDOW_TX:
            new = self.win_acked / self.win_sent
            self.dprr, self.prr = new - self.prr, new
            self.win_sent = self.win_acked = 0
        self._quality()
        self.update_state(now)

    @property
    def cost(self):
        if self.state == "HEALTHY":
            return COST_HEALTHY
        if self.state == "DEGRADING":
            return COST_HEALTHY + max(0.0, COST_SCALE * (1.0 - self.q_pred))
        return INF


# --------------------------------------------------------------------------- one board


class SimPort:
    """Serial-port-like object (read / in_waiting / write / close) for SerialReader."""

    def __init__(self, board, sim):
        self.board, self.sim = board, sim
        self._buf = bytearray()
        self._cv = threading.Condition()

    def push(self, text):
        with self._cv:
            self._buf += text.encode("ascii")
            self._cv.notify()

    @property
    def in_waiting(self):
        return len(self._buf)

    def read(self, n):
        with self._cv:
            if not self._buf:
                self._cv.wait(0.2)
            out = bytes(self._buf[:n])
            del self._buf[:n]
            return out

    def write(self, data):
        self.sim.on_input(self.board.node, data)

    def close(self):
        pass


class SimBoard:
    def __init__(self, node, sim):
        self.node = node
        self.port = SimPort(self, sim)
        self.links = {}                     # nbr -> SimLink
        self.reported = {}                  # nbr -> last state announced with [LINK]
        self.routes = {}                    # dest -> (next, cost, prio_next, hops)
        self.dest = DEFAULT_DEST_ID
        self.prio = False

    def out(self, line):
        self.port.push(line + "\n")

    def usable(self, n):
        link = self.links.get(n)
        return link is not None and link.state != "DEAD"

    def print_tables(self):
        self.out("---- node %u ----" % self.node)
        for i in sorted(self.links):
            s = self.links[i]
            self.out("  nbr %u  %-9s ewma %.1f  slope %.2f  PRR %.2f dPRR %.2f  q %u%% q_pred %.2f"
                     % (i, s.state, s.ewma, s.slope, s.prr, s.dprr, s.q, s.q_pred))
        for d in sorted(self.routes):
            nxt, cost, prio, hops = self.routes[d]
            self.out("  route -> %u  next %u (cost %.2f) | priority next %u (hops %u)" % (d, nxt, cost, prio, hops))


# --------------------------------------------------------------------------- the simulated mesh


class SimMesh:
    def __init__(self, seed=None):
        self.rng = random.Random(seed)
        self.t = 0.0
        self.boards = {n: SimBoard(n, self) for n in NODES}
        self.ports = [f"SIM{n}" for n in NODES]
        self.phase = {(a, b): self.rng.uniform(0, 6.28) for a, b, _ in LINKS}
        self.lock = threading.Lock()
        self._thread = None
        for b in self.boards.values():
            b.out("Node %u ready. Type 0-5 = destination, p = toggle priority." % b.node)

    # ---- SerialReader opener: "SIM2" -> board 2's port, starts the clock on first use
    def open(self, port):
        node = int(port[3:])
        if self._thread is None:
            self._thread = threading.Thread(target=self._run, name="sim-clock", daemon=True)
            self._thread.start()
        return self.boards[node].port

    def _run(self):
        while True:
            start = time.monotonic()
            with self.lock:
                self.step()
            time.sleep(max(0.0, HEARTBEAT_S - (time.monotonic() - start)))

    def on_input(self, node, data):
        with self.lock:
            b = self.boards[node]
            for c in data.decode("ascii", errors="ignore"):
                if "0" <= c < str(MAX_NODES):
                    b.dest = int(c)
                    b.out("destination = %u" % b.dest)
                elif c == "p":
                    b.prio = not b.prio
                    b.out("priority = %s" % ("ON" if b.prio else "OFF"))

    # ---- radio
    def rssi(self, a, b):
        key = (min(a, b), max(a, b))
        base = next(r for x, y, r in LINKS if (min(x, y), max(x, y)) == key)
        return base + fade(a, b, self.t) + 2.0 * math.sin(self.t / 7.0 + self.phase[key]) + self.rng.gauss(0, 1.2)

    def received(self, rssi):
        return self.rng.random() < 1.0 / (1.0 + math.exp(-(rssi - SENS) / K))

    # ---- one 0.5 s step (heartbeat interval)
    def step(self):
        self.t += HEARTBEAT_S
        now = self.t
        for a, b, _ in LINKS:
            for src, dst in ((a, b), (b, a)):
                r = self.rssi(src, dst)
                if self.received(r):                    # heartbeat / hello from src heard by dst
                    rx = self.boards[dst]
                    if src in rx.links:
                        rx.links[src].on_rx(int(round(r)), now)
                    else:
                        rx.links[src] = SimLink(int(round(r)), now)
                        rx.reported[src] = "DEAD"     # firmware: forces a "link up" event
                tx = self.boards[src]
                if tx.usable(dst):                      # unicast heartbeat, acked or not
                    tx.links[dst].on_tx(self.received(r), now)
        for b in self.boards.values():
            for n, link in b.links.items():
                link.update_state(now)
                if link.state != b.reported[n]:
                    b.out("[LINK] node %u: %s -> %s  (ewma %.1f dBm, slope %.2f dB/s, PRR %.2f, q %u%%, q_pred %.2f)"
                          % (n, b.reported[n], link.state, link.ewma, link.slope, link.prr, link.q, link.q_pred))
                    b.reported[n] = link.state
        self.route()
        if abs(now / PRINT_S - round(now / PRINT_S)) < 1e-9:
            for b in self.boards.values():
                b.print_tables()
            for b in self.boards.values():
                if b.dest != b.node:
                    self.send(b.node, b.dest, b.prio)

    # ---- routing.cpp rules, computed from a global view
    def route(self):
        for d in NODES:
            dist = {d: 0.0}                             # cost to d, Dijkstra over each board's own link costs
            hops = {d: 0}
            frontier = [d]
            while frontier:                             # hop counts (BFS) over usable links
                nxt = []
                for v in frontier:
                    for u in NODES:
                        if u not in hops and self.boards[u].usable(v):
                            hops[u] = hops[v] + 1
                            nxt.append(u)
                frontier = nxt
            todo = set(NODES)
            while todo:
                v = min(todo, key=lambda x: dist.get(x, INF))
                todo.discard(v)
                if dist.get(v, INF) == INF:
                    break
                for u in todo:
                    link = self.boards[u].links.get(v)
                    if link and link.cost < INF and dist[v] + link.cost < dist.get(u, INF):
                        dist[u] = dist[v] + link.cost
            for b in self.boards.values():
                if b.node == d:
                    continue
                best, best_c, prio, prio_h, prio_c = None, INF, None, None, INF
                for n, link in b.links.items():
                    c = link.cost + dist.get(n, INF)
                    if c < best_c:
                        best, best_c = n, c
                    h = hops.get(n)
                    if link.cost < INF and h is not None and (prio is None or h + 1 < prio_h
                                                              or (h + 1 == prio_h and c < prio_c)):
                        prio, prio_h, prio_c = n, h + 1, c
                if best is None or best_c == INF:
                    b.routes.pop(d, None)
                    continue
                old = b.routes.get(d)
                if old and old[0] != best:              # eq. 15 hysteresis
                    cur = b.links.get(old[0])
                    cc = cur.cost + dist.get(old[0], INF) if cur else INF
                    if cc < INF and cc - best_c <= ROUTE_HYST_MARGIN:
                        best, best_c = old[0], cc
                b.routes[d] = (best, best_c, prio if prio is not None else best, prio_h or 1)

    # ---- data plane (mesh_node.ino forwardData / handleRx)
    def send(self, src, dest, prio):
        node, ttl = src, MAX_TTL
        while ttl > 0:
            b = self.boards[node]
            r = b.routes.get(dest)
            nh = None if r is None else (r[2] if prio else r[0])
            if nh is None or not b.usable(nh):
                b.out("[FWD] no route to %u" % dest)
                return
            b.out("[FWD] dest %u via %u%s" % (dest, nh, " (PRIORITY)" if prio else ""))
            if not self.received(self.rssi(node, nh)):
                return                                  # lost on the air; the firmware prints nothing
            node, ttl = nh, ttl - 1
            if node == dest:
                s1, s2 = self.rng.randint(0, 4095), self.rng.randint(0, 4095)   # floating pins: noise
                self.boards[dest].out("[DATA] from %u  s1=%d s2=%d  anomaly=0x%02X/0x%02X%s"
                                      % (src, s1, s2, 0, 0, "  PRIORITY" if prio else ""))
                return

    def drain(self, node):
        """Take everything board `node` has printed (tests)."""
        port = self.boards[node].port
        with port._cv:
            out = port._buf.decode("ascii")
            port._buf.clear()
        return out

#!/usr/bin/env python3
"""meshctl - live terminal dashboard for a 4-node hybrid ESP-NOW / LoRa mesh.

Nodes 1 and 2 carry both radios. The N1<->N2 hop prefers ESP-NOW and fails over
to LoRa when the link health (RSSI EWMA + loss) goes critical, then fails back
once ESP-NOW has been healthy for a while.

Run:   pip install rich && python3 mesh_dashboard.py
Keys:  q quit | p pause | f toggle ESP-NOW N1-N2 interference | g toggle LoRa N1-N2 interference
Best viewed in a terminal of at least 140x42.
"""
from __future__ import annotations

import heapq
import math
import os
import random
import select
import sys
import time
from collections import deque

try:
    from rich import box
    from rich.console import Console
    from rich.layout import Layout
    from rich.live import Live
    from rich.panel import Panel
    from rich.table import Table
    from rich.text import Text
except ImportError:
    print("Missing dependency: pip install rich", file=sys.stderr)
    raise SystemExit(2)

try:
    import termios
    import tty
except ImportError:  # non-POSIX
    termios = None

# --------------------------------------------------------------------------- config
TICK = 0.25                 # seconds per update
EWMA_ALPHA = 0.20           # RSSI smoothing:  ewma = a*rssi + (1-a)*ewma
LOSS_ALPHA = 0.08           # beacon-loss smoothing
CONFIRM_TICKS = 3           # ticks a new link state must persist before it is announced
SWITCH_CRIT_TICKS = 4       # critical ticks on the active radio before switching
FAILBACK_TICKS = 40         # healthy ticks on the preferred radio before failing back (10 s)
DOWN_MISS_TICKS = 12        # ticks without any received beacon -> link DOWN
CYCLE = 80.0                # seconds in the interference pattern

PROTO = {
    "ESP-NOW": dict(floor=-90, good=-65, sens=-88, k=2.5, lat=(3, 9), color="green", tag="W"),
    "LoRa":    dict(floor=-118, good=-95, sens=-112, k=3.0, lat=(85, 140), color="magenta", tag="L"),
}

NODES = {
    1: dict(role="GATEWAY", radios="ESP-NOW + LoRa"),
    2: dict(role="RELAY",   radios="ESP-NOW + LoRa"),
    3: dict(role="RELAY",   radios="ESP-NOW"),
    4: dict(role="LEAF",    radios="ESP-NOW"),
}

# (a, b, radio, nominal RSSI dBm)
LINK_DEFS = [
    (1, 2, "ESP-NOW", -58),
    (1, 2, "LoRa",    -92),
    (2, 3, "ESP-NOW", -62),
    (3, 4, "ESP-NOW", -64),
    (1, 3, "ESP-NOW", -76),
    (2, 4, "ESP-NOW", -75),
]

STATE_RANK = {"GOOD": 0, "DEGRADED": 1, "CRITICAL": 2, "DOWN": 3}
STATE_STYLE = {"GOOD": "green", "DEGRADED": "yellow", "CRITICAL": "bold red", "DOWN": "bold white on red"}
LEVEL_STYLE = {"INFO": "cyan", "OK": "green", "WARN": "yellow", "CRIT": "bold red",
               "SWITCH": "bold magenta", "ROUTE": "bright_blue"}
BLOCKS = "▁▂▃▄▅▆▇█"


def trap(x, a, b, c, d, depth):
    """Trapezoid pulse: ramps 0->depth over [a,b], holds to c, ramps back by d."""
    if x <= a or x >= d:
        return 0.0
    if x < b:
        return depth * (x - a) / (b - a)
    if x <= c:
        return depth
    return depth * (d - x) / (d - c)


def fmt_path(path):
    return "→".join(f"N{x}" for x in path)


def spark(vals, lo, hi, width=14):
    out = ""
    for v in list(vals)[-width:]:
        frac = max(0.0, min(1.0, (v - lo) / (hi - lo)))
        out += BLOCKS[int(frac * 7)]
    return out


# --------------------------------------------------------------------------- model
class RadioLink:
    def __init__(self, a, b, proto, base):
        self.a, self.b, self.proto, self.base = a, b, proto, base
        self.raw = base
        self.ewma = float(base)
        self.loss = 0.0
        self.pdr = 1.0
        self.pending = None
        self.pn = 0
        self.miss = 0
        self.phase = random.uniform(0, 6.28)
        self.hist = deque([float(base)] * 14, maxlen=14)
        p = PROTO[proto]
        comp = max(0.0, min(1.0, (base - p["floor"]) / (p["good"] - p["floor"])))
        self.health = 100.0 * comp
        self.state = "GOOD" if self.health >= 70 else "DEGRADED" if self.health >= 40 else "CRITICAL"

    @property
    def name(self):
        return f"N{self.a}↔N{self.b}"


class NodeStat:
    def __init__(self):
        self.tx = self.rx = self.fwd = self.drop = self.delivered = 0


class Mesh:
    def __init__(self):
        self.t = 6.0
        self.t0 = self.t
        self.links = [RadioLink(*d) for d in LINK_DEFS]
        self.by_key = {(l.a, l.b, l.proto): l for l in self.links}
        self.adj = {n: set() for n in NODES}
        for l in self.links:
            self.adj[l.a].add(l.b)
            self.adj[l.b].add(l.a)
        self.active = {(1, 2): "ESP-NOW"}          # dual-radio pairs -> radio in use
        self.crit_n = {(1, 2): 0}
        self.good_n = {(1, 2): 0}
        self.nodes = {n: NodeStat() for n in NODES}
        self.proto = {p: dict(tx=0, rx=0, drop=0, lat=None) for p in PROTO}
        self.routes = {}
        self.events = deque(maxlen=200)
        self.fault = {"ESP-NOW": False, "LoRa": False}
        self.handovers = 0
        self.route_changes = 0
        self.sent = 0
        self.delivered = 0
        self.e2e_lat = None
        self.window = deque(maxlen=16)
        # neighbor table (as seen by N1): learned only from hello beacons of direct radio neighbors
        self.neigh = {n: dict(seen=None, proto=None, rssi=None, ewma=None) for n in self.adj[1]}
        self.paused = False

        self.log("INFO", "Mesh online · 4 nodes · gateway N1")
        self.log("INFO", "N1↔N2 dual-radio hop: ESP-NOW primary, LoRa standby")
        self.compute_routes(initial=True)

    # ---- helpers
    def log(self, level, msg):
        self.events.append((time.strftime("%H:%M:%S"), level, msg))

    def get(self, a, b, proto):
        return self.by_key.get((min(a, b), max(a, b), proto))

    def edge(self, a, b):
        k = (min(a, b), max(a, b))
        proto = self.active.get(k, "ESP-NOW")
        l = self.by_key.get((k[0], k[1], proto))
        return l if l and l.state != "DOWN" else None

    @staticmethod
    def cost(l):
        return 1.0 + (100.0 - l.health) / 25.0 + (0.6 if l.proto == "LoRa" else 0.0)

    # ---- RF model
    def offset(self, l, tc):
        """Scripted interference (dB) on a link as a function of the cycle time."""
        key = (l.a, l.b, l.proto)
        off = 0.0
        if key == (1, 2, "ESP-NOW"):
            off += trap(tc, 10, 24, 44, 56, -34)
        elif key == (3, 4, "ESP-NOW"):
            off += trap(tc, 60, 64, 68, 72, -18)
        elif key == (1, 2, "LoRa"):
            off += trap(tc, 70, 72, 75, 77, -8)
        if key == (1, 2, "ESP-NOW") and self.fault["ESP-NOW"]:
            off -= 40
        if key == (1, 2, "LoRa") and self.fault["LoRa"]:
            off -= 40
        return off

    def sample_link(self, l):
        p = PROTO[l.proto]
        tc = self.t % CYCLE
        fade = 2.2 * math.sin(2 * math.pi * self.t / 19.0 + l.phase)
        true_rssi = l.base + self.offset(l, tc) + fade + random.gauss(0, 1.3)
        l.pdr = 1.0 / (1.0 + math.exp(-(true_rssi - p["sens"]) / p["k"]))
        ok = random.random() < l.pdr
        if ok:                                   # RSSI is only measurable on a received frame
            l.raw = int(round(true_rssi))
            l.ewma = EWMA_ALPHA * l.raw + (1 - EWMA_ALPHA) * l.ewma
            l.miss = 0
        else:
            l.miss += 1
        l.loss = LOSS_ALPHA * (0.0 if ok else 1.0) + (1 - LOSS_ALPHA) * l.loss
        l.hist.append(l.ewma)

        # health 0..100 from RSSI EWMA position between floor and good, scaled by loss
        if l.miss >= DOWN_MISS_TICKS:
            l.health, raw_state = 0.0, "DOWN"
        else:
            comp = max(0.0, min(1.0, (l.ewma - p["floor"]) / (p["good"] - p["floor"])))
            l.health = 100.0 * comp * (1.0 - min(1.0, l.loss)) ** 1.5
            raw_state = "GOOD" if l.health >= 70 else "DEGRADED" if l.health >= 40 else "CRITICAL"

        if raw_state == l.state:
            l.pending, l.pn = None, 0
            return
        l.pn = l.pn + 1 if raw_state == l.pending else 1
        l.pending = raw_state
        if l.pn >= CONFIRM_TICKS:
            old, l.state, l.pending, l.pn = l.state, raw_state, None, 0
            self.on_state(l, old, raw_state)

    def on_state(self, l, old, new):
        if old in ("CRITICAL", "DOWN") and new in ("CRITICAL", "DOWN"):
            return
        detail = f"EWMA {l.ewma:.0f} dBm · loss {l.loss*100:.0f}% · health {l.health:.0f}"
        msg = f"{l.proto} {l.name} {old} → {new}  ({detail})"
        if STATE_RANK[new] > STATE_RANK[old]:
            self.log("WARN" if new == "DEGRADED" else "CRIT", msg)
        elif new == "GOOD":
            self.log("OK", msg)
        else:
            self.log("INFO", msg)

    # ---- protocol switching
    def evaluate_pairs(self):
        for k, act in list(self.active.items()):
            pref, alt = self.get(*k, "ESP-NOW"), self.get(*k, "LoRa")
            cur, other = (pref, alt) if act == "ESP-NOW" else (alt, pref)
            bad = cur.state in ("CRITICAL", "DOWN")
            self.crit_n[k] = self.crit_n[k] + 1 if bad else 0
            other_ok = other.state not in ("CRITICAL", "DOWN")
            if self.crit_n[k] >= SWITCH_CRIT_TICKS and other_ok:
                self.active[k] = other.proto
                self.handovers += 1
                self.crit_n[k] = self.good_n[k] = 0
                self.log("SWITCH",
                         f"N{k[0]}↔N{k[1]} handover {cur.proto} → {other.proto}  "
                         f"({cur.proto} {cur.ewma:.0f} dBm critical, {other.proto} {other.ewma:.0f} dBm)")
                continue
            if self.crit_n[k] == SWITCH_CRIT_TICKS and not other_ok:
                self.log("CRIT", f"N{k[0]}↔N{k[1]}: both radios unhealthy, no clean alternate")
            if act == "LoRa":
                self.good_n[k] = self.good_n[k] + 1 if pref.state == "GOOD" else 0
                if self.good_n[k] >= FAILBACK_TICKS:
                    self.active[k] = "ESP-NOW"
                    self.handovers += 1
                    self.good_n[k] = 0
                    self.log("SWITCH",
                             f"N{k[0]}↔N{k[1]} failback LoRa → ESP-NOW  "
                             f"(ESP-NOW stable at {pref.ewma:.0f} dBm for {FAILBACK_TICKS*TICK:.0f}s)")

    # ---- routing
    def compute_routes(self, initial=False):
        dist, parent, pq = {1: 0.0}, {}, [(0.0, 1)]
        while pq:
            d, u = heapq.heappop(pq)
            if d > dist.get(u, 1e9):
                continue
            for v in self.adj[u]:
                l = self.edge(u, v)
                if not l:
                    continue
                nd = d + self.cost(l)
                if nd < dist.get(v, 1e9) - 1e-9:
                    dist[v], parent[v] = nd, u
                    heapq.heappush(pq, (nd, v))

        for src in (2, 3, 4):
            new = None
            if src in dist:
                new = [src]
                while new[-1] != 1:
                    new.append(parent[new[-1]])
            cur = self.routes.get(src)
            keep = None
            if cur:
                c = self.path_cost(cur["path"])
                if c is not None and (new is None or c <= dist[src] + 0.75):
                    keep = c
            if keep is not None:                         # hysteresis: stay on the current path
                cur["cost"] = keep
                cur["proto"] = self.edge(cur["path"][0], cur["path"][1]).proto
                continue
            if new is None:
                if cur is not None:
                    self.log("CRIT", f"Route N{src}→N1 lost: no usable path")
                    self.route_changes += 1
                    del self.routes[src]
                continue
            first = self.edge(new[0], new[1])
            route = dict(path=new, cost=dist[src], proto=first.proto,
                         seq=(cur["seq"] + 1 if cur else 1), changed=self.t)
            if cur and not initial:
                self.route_changes += 1
                self.log("ROUTE", f"Route N{src}→N1: {fmt_path(cur['path'])} ⇒ {fmt_path(new)}  "
                                  f"(metric {route['cost']:.1f})")
            elif not cur and not initial:
                self.route_changes += 1
                self.log("ROUTE", f"Route N{src}→N1 restored: {fmt_path(new)}")
            self.routes[src] = route

    def path_cost(self, path):
        total = 0.0
        for u, v in zip(path, path[1:]):
            l = self.edge(u, v)
            if l is None:
                return None
            total += self.cost(l)
        return total

    # ---- data plane
    def traffic(self):
        hops = deliv = 0
        for src in (2, 3, 4):
            if random.random() > 0.45:
                continue
            n = self.nodes[src]
            n.tx += 1
            self.sent += 1
            r = self.routes.get(src)
            if not r:
                n.drop += 1
                continue
            lat, ok, last = 0.0, True, None
            for u, v in zip(r["path"], r["path"][1:]):
                l = self.edge(u, v)
                if l is None:
                    self.nodes[u].drop += 1
                    ok = False
                    break
                ps = self.proto[l.proto]
                ps["tx"] += 1
                hops += 1
                if random.random() < 1.0 - (1.0 - l.pdr) ** 2:        # one retry per hop
                    ps["rx"] += 1
                    self.nodes[v].rx += 1
                    if v != 1:
                        self.nodes[v].fwd += 1
                    hl = random.uniform(*PROTO[l.proto]["lat"])
                    lat += hl
                    ps["lat"] = hl if ps["lat"] is None else 0.9 * ps["lat"] + 0.1 * hl
                    last = (u, l.proto)
                else:
                    ps["drop"] += 1
                    self.nodes[u].drop += 1
                    ok = False
                    break
            if ok:
                deliv += 1
                self.delivered += 1
                n.delivered += 1
                n.rx += 1                                   # ACK back from the gateway
                self.nodes[1].tx += 1
                self.e2e_lat = lat if self.e2e_lat is None else 0.9 * self.e2e_lat + 0.1 * lat
        self.window.append((hops, deliv))

    def refresh_neighbors(self):
        """A hello beacon received from a direct neighbor refreshes its entry."""
        for v, e in self.neigh.items():
            l = self.edge(1, v)
            if l and l.miss == 0:
                e.update(seen=self.t, proto=l.proto, rssi=l.raw, ewma=l.ewma)

    def step(self):
        if self.paused:
            return
        self.t += TICK
        for l in self.links:
            self.sample_link(l)
        self.evaluate_pairs()
        self.compute_routes()
        self.traffic()
        self.refresh_neighbors()

    def used_edges(self):
        used = set()
        for r in self.routes.values():
            for u, v in zip(r["path"], r["path"][1:]):
                used.add((min(u, v), max(u, v), self.active.get((min(u, v), max(u, v)), "ESP-NOW")))
        return used


# --------------------------------------------------------------------------- rendering
def bar(v, width=10):
    n = int(round(v / 100.0 * width))
    return "█" * n + "░" * (width - n)


def nodes_panel(m):
    t = Table(box=box.SIMPLE_HEAVY, expand=True, header_style="bold bright_cyan", pad_edge=False)
    for col, kw in (("NODE", dict(width=4)), ("ROLE", dict(width=8)), ("RADIOS", dict(min_width=14)),
                    ("STATUS", dict(width=9)), ("TX", dict(justify="right")), ("RX", dict(justify="right")),
                    ("FWD", dict(justify="right")), ("DROP", dict(justify="right")), ("PDR", dict(justify="right"))):
        t.add_column(col, **kw)
    for n, info in NODES.items():
        alive = any(m.edge(n, v) for v in m.adj[n])
        st = m.nodes[n]
        if n == 1:
            pdr = f"{100.0 * m.delivered / m.sent:.1f}%" if m.sent else "—"
        else:
            pdr = f"{100.0 * st.delivered / st.tx:.1f}%" if st.tx else "—"
        t.add_row(f"N{n}", info["role"], info["radios"],
                  "[green]● ONLINE[/]" if alive else "[bold red]○ ISOLATED[/]",
                  str(st.tx), str(st.rx), str(st.fwd), str(st.drop), pdr)
    return Panel(t, title="[bold]NODES[/]  [dim]packets per node · PDR = delivered / originated[/]", border_style="cyan")


def links_panel(m):
    t = Table(box=box.SIMPLE_HEAVY, expand=True, header_style="bold bright_cyan", pad_edge=False)
    for col, kw in (("LINK", dict(width=7)), ("RADIO", dict(width=8)), ("ROLE", dict(width=8)),
                    ("RSSI", dict(justify="right", width=5)), ("EWMA", dict(justify="right", width=7)),
                    ("TREND", dict(width=14)), ("LOSS", dict(justify="right", width=6)),
                    ("HEALTH", dict(min_width=15)), ("STATE", dict(width=9))):
        t.add_column(col, **kw)
    used = m.used_edges()
    for l in sorted(m.links, key=lambda x: (x.a, x.b, x.proto)):
        dual = (l.a, l.b) in m.active
        if dual:
            role = "ACTIVE" if m.active[(l.a, l.b)] == l.proto else "STANDBY"
        else:
            role = "ACTIVE" if (l.a, l.b, l.proto) in used else "IDLE"
        rstyle = {"ACTIVE": "bold white", "STANDBY": "dim", "IDLE": "dim"}[role]
        p = PROTO[l.proto]
        sst = STATE_STYLE[l.state]
        raw = f"{l.raw}" + ("·" if l.miss > 2 else "")
        t.add_row(l.name, f"[{p['color']}]{l.proto}[/]", f"[{rstyle}]{role}[/]", raw,
                  f"{l.ewma:.1f}", f"[{sst}]{spark(l.hist, p['floor'], p['good'])}[/]",
                  f"{l.loss*100:.1f}%", f"[{sst}]{bar(l.health)}[/] {l.health:3.0f}",
                  f"[{sst}]{l.state}[/]")
    return Panel(t, title="[bold]LINK HEALTH[/]  [dim]RSSI dBm · EWMA α=0.20 · ·=no recent frame[/]", border_style="cyan")


def routes_panel(m):
    t = Table(box=box.SIMPLE_HEAVY, expand=True, header_style="bold bright_cyan", pad_edge=False)
    for col, kw in (("DEST", dict(width=5)), ("NEXT", dict(width=5)), ("IFACE", dict(width=8)),
                    ("HOPS", dict(justify="right", width=4)), ("PATH", dict(min_width=14)),
                    ("METRIC", dict(justify="right", width=6)), ("SEQ", dict(justify="right", width=3)),
                    ("AGE", dict(justify="right", width=6))):
        t.add_column(col, **kw)
    t.add_row("N1", "local", "—", "0", "N1", "0.0", "—", "—")
    for src in (2, 3, 4):
        r = m.routes.get(src)
        if not r:
            t.add_row(f"[bold red]N{src}[/]", "—", "—", "—", "[bold red]no route[/]", "—", "—", "—")
            continue
        col = PROTO[r["proto"]]["color"]
        t.add_row(f"N{src}", f"N{r['path'][1]}", f"[{col}]{r['proto']}[/]", str(len(r["path"]) - 1),
                  fmt_path(r["path"]), f"{r['cost']:.1f}", str(r["seq"]), f"{m.t - r['changed']:.0f}s")
    return Panel(t, title="[bold]ROUTING TABLE[/]  [dim]to gateway N1 · metric = link cost[/]", border_style="blue")


def neighbor_panel(m):
    t = Table(box=box.SIMPLE_HEAVY, expand=True, header_style="bold bright_cyan", pad_edge=False)
    for col, kw in (("ID", dict(width=3)), ("MAC", dict(min_width=17)), ("IFACE", dict(width=8)),
                    ("RSSI", dict(justify="right", width=5)), ("EWMA", dict(justify="right", width=6)),
                    ("AGE", dict(justify="right", width=5)), ("STATE", dict(width=9))):
        t.add_column(col, **kw)
    for n in sorted(m.neigh):
        e = m.neigh[n]
        mac = f"24:6F:28:A{n}:5C:{n:02X}"
        if e["seen"] is None:
            t.add_row(f"N{n}", mac, "—", "—", "—", "—", "[dim]UNKNOWN[/]")
            continue
        age = m.t - e["seen"]
        state = "[green]REACHABLE[/]" if age < 2 else "[yellow]STALE[/]" if age < 6 else "[bold red]FAILED[/]"
        col = PROTO[e["proto"]]["color"]
        t.add_row(f"N{n}", mac, f"[{col}]{e['proto']}[/]", str(e["rssi"]), f"{e['ewma']:.1f}",
                  f"{age:.1f}s", state)
    return Panel(t, title="[bold]NEIGHBOR TABLE[/]  [dim]N1 · learned from hello beacons[/]", border_style="blue")


def packets_panel(m):
    t = Table(box=box.SIMPLE_HEAVY, expand=True, show_header=False, padding=(0, 1))
    t.add_column(style="dim")
    t.add_column(justify="right", style="bold")
    tx = sum(p["tx"] for p in m.proto.values())
    rx = sum(p["rx"] for p in m.proto.values())
    dr = sum(p["drop"] for p in m.proto.values())
    win_t = max(1, len(m.window)) * TICK
    rate = sum(h for h, _ in m.window) / win_t
    t.add_row("Hop TX / RX / drop", f"{tx} / {rx} / {dr}")
    t.add_row("End-to-end delivery", f"{100.0*m.delivered/m.sent:.1f}%" if m.sent else "—")
    t.add_row("Throughput", f"{rate:.1f} pkt/s")
    t.add_row("E2E latency (EWMA)", f"{m.e2e_lat:.0f} ms" if m.e2e_lat else "—")
    for name, p in m.proto.items():
        c = PROTO[name]["color"]
        loss = f"{100.0*p['drop']/p['tx']:.1f}%" if p["tx"] else "—"
        lat = f"{p['lat']:.0f} ms" if p["lat"] else "—"
        t.add_row(f"[{c}]{name}[/] tx/rx · loss", f"{p['tx']}/{p['rx']} · {loss}")
        t.add_row(f"[{c}]{name}[/] hop latency", lat)
    t.add_row("Handovers / route changes", f"{m.handovers} / {m.route_changes}")
    return Panel(t, title="[bold]PACKET STATS[/]", border_style="magenta")


def events_panel(m):
    t = Table(box=box.SIMPLE, expand=True, show_header=False, padding=(0, 1))
    t.add_column(width=8, style="dim")
    t.add_column(width=6)
    t.add_column(overflow="fold")
    for stamp, level, msg in list(m.events)[::-1][:40]:
        t.add_row(stamp, f"[{LEVEL_STYLE[level]}]{level}[/]", f"[{LEVEL_STYLE[level]}]{msg}[/]"
                  if level in ("CRIT", "SWITCH") else msg)
    return Panel(t, title="[bold]EVENT LOG[/]  [dim]newest first[/]", border_style="cyan")


def topology_panel(m):
    out = Text()
    for n in (1, 2, 3, 4):
        node = f"N{n}"
        out.append(f" ● {node}", style="bold bright_white")
        out.append(f"  {NODES[n]['role'].lower()}\n", style="dim")
        if n == 4:
            break
        l = m.edge(n, n + 1)
        if l is None:
            out.append("  ╳  link down\n", style="bold white on red")
            continue
        c = PROTO[l.proto]["color"]
        glyph = "║" if l.proto == "LoRa" else "│"
        out.append(f"  {glyph} ", style=c)
        out.append(f"{l.proto}", style=f"bold {c}")
        out.append(f"  {l.ewma:.0f} dBm  ", style=STATE_STYLE[l.state])
        out.append(f"{l.state}\n", style=STATE_STYLE[l.state])
    out.append("\n W/ESP-NOW = green   LoRa = magenta", style="dim")
    return Panel(out, title="[bold]ACTIVE CHAIN[/]", border_style="blue")


def render(m, console):
    root = Layout()
    root.split_column(Layout(name="head", size=3), Layout(name="top", ratio=10),
                      Layout(name="mid", ratio=9), Layout(name="bot", ratio=9), Layout(name="foot", size=3))
    online = sum(1 for n in NODES if any(m.edge(n, v) for v in m.adj[n]))
    worst = max((STATE_RANK[l.state] for l in m.links if (l.a, l.b, l.proto) in m.used_edges()), default=0)
    mesh_state = ["HEALTHY", "DEGRADED", "CRITICAL", "DOWN"][worst]
    mcol = ["green", "yellow", "red", "red"][worst]
    title = Text("meshctl  /  hybrid ESP-NOW + LoRa mesh", style="bold bright_white")
    title.append(f"   ● MESH {mesh_state}", style=f"bold {mcol}")
    title.append(f"   {online}/{len(NODES)} nodes", style="dim")
    title.append(f"   uptime {int(m.t - m.t0):>4}s", style="dim")
    title.append(f"   {time.strftime('%H:%M:%S')}", style="dim")
    if m.paused:
        title.append("   [PAUSED]", style="bold yellow")
    root["head"].update(Panel(title, border_style=mcol, padding=(0, 1)))

    root["top"].split_row(Layout(nodes_panel(m), ratio=5), Layout(links_panel(m), ratio=8))
    root["mid"].split_row(Layout(routes_panel(m), ratio=5), Layout(neighbor_panel(m), ratio=5),
                          Layout(packets_panel(m), ratio=4))
    root["bot"].split_row(Layout(events_panel(m), ratio=3), Layout(topology_panel(m), ratio=1))
    root["foot"].update(Panel(
        "[bold cyan]q[/] Quit   [bold cyan]p[/] Pause   "
        "[bold cyan]f[/] Toggle ESP-NOW N1↔N2 interference   [bold cyan]g[/] Toggle LoRa N1↔N2 interference",
        border_style="blue", padding=(0, 1)))
    return root


# --------------------------------------------------------------------------- main
def read_keys(fd):
    keys = []
    while select.select([fd], [], [], 0)[0]:
        data = os.read(fd, 32)
        if not data:
            break
        keys.extend(chr(c).lower() for c in data)
    return keys


def main():
    console = Console()
    mesh = Mesh()
    fd, old = None, None
    if termios and sys.stdin.isatty():
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        tty.setcbreak(fd)
    try:
        with Live(render(mesh, console), console=console, screen=True, auto_refresh=False) as live:
            while True:
                started = time.monotonic()
                for k in (read_keys(fd) if fd is not None else []):
                    if k == "q":
                        return
                    if k == "p":
                        mesh.paused = not mesh.paused
                    elif k == "f":
                        mesh.fault["ESP-NOW"] = not mesh.fault["ESP-NOW"]
                    elif k == "g":
                        mesh.fault["LoRa"] = not mesh.fault["LoRa"]
                mesh.step()
                live.update(render(mesh, console), refresh=True)
                time.sleep(max(0.0, TICK - (time.monotonic() - started)))
    except KeyboardInterrupt:
        pass
    finally:
        if fd is not None and old is not None:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)


if __name__ == "__main__":
    main()

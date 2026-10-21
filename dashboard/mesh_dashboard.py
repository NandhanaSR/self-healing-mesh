#!/usr/bin/env python3
"""meshctl - live terminal dashboard for the predictive self-healing ESP-NOW mesh.

Reads each board's serial output (one COM port per board) and shows what the boards
report: link state, ewma, slope, PRR/dPRR, q and q_pred per link, and each board's
next hop / priority next hop per destination. Nothing is recomputed in Python.

Run:   pip install rich pyserial
       python dashboard/mesh_dashboard.py --ports COM3 COM4 COM5 COM6
       python dashboard/mesh_dashboard.py --replay dashboard/samples/mesh4/node*.txt   (one frame, no boards)
Keys:  Ctrl+C quit
Best viewed in Windows Terminal of at least 140x42.
"""
from __future__ import annotations

import argparse
import heapq
import math
import random
import sys
import time
from collections import deque

try:
    from rich import box
    from rich.console import Console, Group
    from rich.layout import Layout
    from rich.live import Live
    from rich.panel import Panel
    from rich.table import Table
    from rich.text import Text
except ImportError:
    print("Missing dependency: pip install rich", file=sys.stderr)
    raise SystemExit(2)

from live_model import MAX_NODES, LiveMesh, replay, replay_now

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


# --------------------------------------------------------------------------- rendering (live boards)
# Every number below is as printed by a board (via live_model.LiveMesh); nothing is recomputed.
Q_TH = 0.50                 # config.h: q_pred below this -> DEGRADING (used for colour only)
Q_RECOVER = 0.65            # config.h: back to HEALTHY above this
RSSI_MIN, RSSI_MAX = -85, -40   # config.h: RSSI score range, used to scale the ewma sparkline
DEFAULT_DEST_ID = 1         # config.h: data destination a board starts with

LINK_RANK = {"HEALTHY": 0, "DEGRADING": 1, "DEAD": 2}
LINK_STYLE = {"HEALTHY": "green", "DEGRADING": "yellow", "DEAD": "bold white on red"}
LINK_LETTER = {"HEALTHY": "H", "DEGRADING": "D", "DEAD": "X"}
LIVE_LEVEL_STYLE = {"INFO": "cyan", "OK": "green", "WARN": "yellow", "CRIT": "bold red", "ROUTE": "bright_blue"}


def bar(v, width=10):
    n = int(round(v / 100.0 * width))
    return "█" * n + "░" * (width - n)


def q_style(q_pred):
    return "bold red" if q_pred < Q_TH else "yellow" if q_pred < Q_RECOVER else "green"


def sorted_boards(m):
    return sorted(m.boards.values(), key=lambda b: (b.node_id is None, b.node_id or 0, b.port))


def board_status(b, now):
    if not b.connected and b.table_time is None:
        return "[bold red]○ OFFLN[/]"
    if b.table_time is None:
        return "[dim]… waiting[/]"
    if not b.connected:
        return "[bold red]○ LOST[/]"
    if b.stale(now):
        return "[yellow]◌ STALE[/]"
    return "[green]● LIVE[/]"


def boards_panel(m, now):
    t = Table(box=box.SIMPLE_HEAVY, expand=True, header_style="bold bright_cyan", pad_edge=False, padding=(0, 1, 0, 0), show_edge=False)
    for col, kw in (("NODE", dict(width=4)), ("PORT", dict(min_width=5)), ("STATUS", dict(width=9)),
                    ("DEST", dict(justify="right", width=4)), ("PRIO", dict(justify="right", width=4)),
                    ("FWD", dict(justify="right")), ("NO RT", dict(justify="right", width=5)),
                    ("DATA", dict(justify="right"))):
        t.add_column(col, **kw)
    for b in sorted_boards(m):
        node = f"N{b.node_id}" if b.node_id is not None else "?"
        dest = "?" if b.dest is None else f"N{b.dest}"
        prio = "?" if b.priority is None else ("[bold magenta]ON[/]" if b.priority else "[dim]off[/]")
        no_rt = f"[bold red]{b.no_route}[/]" if b.no_route else "0"
        t.add_row(node, b.port, board_status(b, now), dest, prio, str(b.fwd), no_rt, str(b.delivered))
    return Panel(t, title="[bold]BOARDS[/]  [dim]counts = serial lines · ? = not printed yet[/]",
                 border_style="cyan")


def links_panel(m, now):
    t = Table(box=box.SIMPLE_HEAVY, expand=True, header_style="bold bright_cyan", pad_edge=False, padding=(0, 1, 0, 0), show_edge=False)
    for col, kw in (("LINK", dict(no_wrap=True)), ("STATE", dict(width=9)), ("EWMA", dict(justify="right", width=6)),
                    ("TREND", dict(width=8)), ("SLOPE", dict(justify="right")),
                    ("PRR", dict(justify="right")), ("dPRR", dict(justify="right")),
                    ("HEALTH q", dict(width=10)), ("Q_PRED", dict(width=11))):
        t.add_column(col, **kw)
    for b in sorted_boards(m):
        dim = " dim" if b.stale(now) else ""
        for n in sorted(b.neighbors.values(), key=lambda x: x.nbr):
            sst = LINK_STYLE[n.state]
            qst = q_style(n.q_pred)
            slope = f"[red]{n.slope:.2f}[/]" if n.slope < 0 else f"{n.slope:.2f}"
            dprr = f"[red]{n.dprr:.2f}[/]" if n.dprr < 0 else f"{n.dprr:.2f}"
            t.add_row(f"[bold{dim}]{b.label}→N{n.nbr}[/]", f"[{sst}]{n.state}[/]", f"{n.ewma:.1f}",
                      f"[{sst}]{spark(b.ewma_hist.get(n.nbr, ()), RSSI_MIN, RSSI_MAX, width=8)}[/]",
                      slope, f"{n.prr:.2f}", dprr, f"[{sst}]{bar(n.q, width=6)}[/] {n.q:3d}",
                      f"[{qst}]{n.q_pred:.2f} {spark(b.qpred_hist.get(n.nbr, ()), 0.0, 1.0, width=6)}[/]")
    return Panel(t if t.row_count else Text("no link data yet", style="dim"), title=f"[bold]LINK HEALTH[/]  [dim]as measured by each board · ewma dBm · slope dB/s · "
                          f"q_pred < {Q_TH:.2f} → DEGRADING[/]",
                 border_style="cyan")


def path_text(m, nodes, status):
    """N0 ─0.91→ N2 ─0.94→ N1, each hop coloured by the link state the sending board measures."""
    boards = m.by_id()
    out = Text()
    for i, n in enumerate(nodes):
        out.append(f"N{n}", style="bold bright_white")
        if i + 1 < len(nodes):
            b = boards.get(n)
            link = b.neighbors.get(nodes[i + 1]) if b else None
            if link:
                out.append(f" ─{link.q_pred:.2f}→ ", style=LINK_STYLE[link.state])
            else:
                out.append(" ─?→ ", style="dim")
    if status == "no-route":
        out.append("  ✗ no route", style="bold red")
    elif status == "unknown":
        out.append("  … board not connected", style="dim")
    elif status == "loop":
        out.append("  ⟲ loop", style="bold red")
    return out


def routes_panel(m):
    t = Table(box=box.SIMPLE_HEAVY, expand=True, header_style="bold bright_cyan", pad_edge=False, collapse_padding=True)
    for col, kw in (("AT", dict(width=3)), ("DEST", dict(width=5)), ("NEXT", dict(width=4)),
                    ("COST", dict(justify="right", width=5)), ("PRIO", dict(width=4)),
                    ("HOPS", dict(justify="right", width=4)), ("PATH (next hops)", dict(min_width=16, overflow="fold"))):
        t.add_column(col, **kw)
    for b in sorted_boards(m):
        if b.table_time is None:
            continue
        if not b.routes:
            t.add_row(b.label, "[dim]no routes[/]", "", "", "", "", "")
        for r in sorted(b.routes.values(), key=lambda x: x.dest):
            star = "★" if r.dest == b.dest else " "
            prio = f"[bold magenta]N{r.prio_next}[/]" if r.prio_next != r.next else f"N{r.prio_next}"
            t.add_row(b.label, f"{star}N{r.dest}", f"N{r.next}", f"{r.cost:.2f}", prio, str(r.hops),
                      path_text(m, *m.path(b.node_id, r.dest)))
    return Panel(t if t.row_count else Text("no routing tables yet", style="dim"), title="[bold]ROUTING[/]  [dim]each board's table · PRIO = priority next hop · ★ = data destination[/]", border_style="blue")


def neighbors_panel(m):
    t = Table(box=box.SIMPLE_HEAVY, header_style="bold bright_cyan", pad_edge=False, padding=(0, 0), show_edge=False)
    t.add_column("", width=3)
    for i in range(MAX_NODES):
        t.add_column(f"N{i}", justify="center")
    for b in sorted_boards(m):
        if b.node_id is None:
            continue
        cells = []
        for i in range(MAX_NODES):
            n = b.neighbors.get(i)
            if i == b.node_id:
                cells.append("[dim]—[/]")
            elif n:
                cells.append(f"[{LINK_STYLE[n.state]}]{LINK_LETTER[n.state]}[/]")
            else:
                cells.append("[dim]·[/]")
        t.add_row(f"[bold]{b.label}[/]", *cells)
    legend = Text("\nrow sees col\nH healthy · D degrading\nX dead", style="dim")
    return Panel(Group(t, legend), title="[bold]NEIGHBORS[/]", border_style="blue")


def traffic_panel(m):
    t = Table(box=box.SIMPLE_HEAVY, expand=True, show_header=False, padding=(0, 1), show_edge=False)
    t.add_column(style="dim")
    t.add_column(justify="right", style="bold")
    boards = list(m.boards.values())
    fwd = sum(b.fwd for b in boards)
    t.add_row("FWD (prio)", f"{fwd} ({sum(b.fwd_prio for b in boards)})")
    t.add_row("No route", str(sum(b.no_route for b in boards)))
    for b in sorted_boards(m):
        if b.delivered:
            per_src = " ".join(f"N{s}:{c}" for s, c in sorted(b.delivered_from.items()))
            t.add_row(f"DATA at {b.label}", str(b.delivered))
            t.add_row("  from", f"[dim]{per_src}[/]")
    t.add_row("Link changes", str(m.link_changes))
    t.add_row("Route changes", str(m.route_changes))
    return Panel(t, title="[bold]TRAFFIC[/]  [dim]serial lines since start[/]", border_style="magenta")


def events_panel(m):
    t = Table(box=box.SIMPLE, expand=True, show_header=False, padding=(0, 1))
    t.add_column(width=8, style="dim")
    t.add_column(width=6)
    t.add_column(overflow="fold")
    for stamp, level, msg in list(m.events)[::-1][:40]:
        style = LIVE_LEVEL_STYLE.get(level, "white")
        t.add_row(stamp, f"[{style}]{level}[/]", Text(msg, style=style if level == "CRIT" else ""))
    return Panel(t, title="[bold]EVENT LOG[/]  [dim]newest first[/]", border_style="cyan")


def paths_panel(m):
    out = Text()
    for b in sorted_boards(m):
        if b.node_id is None or b.table_time is None:
            continue
        dest = b.dest if b.dest is not None else DEFAULT_DEST_ID
        if dest == b.node_id:
            continue
        guess = "" if b.dest is not None else "?"       # dest not printed yet: firmware default assumed
        out.append(f" {b.label}→N{dest}{guess}  ", style="bold")
        out.append_text(path_text(m, *m.path(b.node_id, dest)))
        out.append("\n")
        prio_nodes, prio_status = m.path(b.node_id, dest, priority=True)
        if prio_nodes != m.path(b.node_id, dest)[0] or b.priority:
            out.append("   prio  ", style="magenta" if b.priority else "dim magenta")
            out.append_text(path_text(m, prio_nodes, prio_status))
            out.append("\n")
    if not out.plain:
        out.append(" waiting for routing tables…", style="dim")
    out.append(f"\n number = q_pred of the hop · ? = destination not printed yet (default N{DEFAULT_DEST_ID})",
               style="dim")
    return Panel(out, title="[bold]ACTIVE PATHS[/]", border_style="blue")


def render(m, now, mode="LIVE"):
    root = Layout()
    root.split_column(Layout(name="head", size=3), Layout(name="top", ratio=10),
                      Layout(name="mid", ratio=9), Layout(name="bot", ratio=9), Layout(name="foot", size=3))
    boards = list(m.boards.values())
    live = sum(1 for b in boards if b.connected and not b.stale(now)) if mode == "LIVE" else \
        sum(1 for b in boards if b.table_time is not None)
    states = [n.state for b in boards if not b.stale(now) for n in b.neighbors.values()]
    worst = max((LINK_RANK[s] for s in states), default=None)
    worst_name = "—" if worst is None else ["HEALTHY", "DEGRADING", "DEAD"][worst]
    wcol = "dim" if worst is None else ["green", "yellow", "red"][worst]
    title = Text("meshctl  /  predictive ESP-NOW mesh", style="bold bright_white")
    if mode == "LIVE":
        title.append("   ● LIVE", style="bold green")
    else:
        title.append(f"   ▶ {mode}", style="bold black on yellow")
    title.append(f"   worst link {worst_name}", style=f"bold {wcol}")
    title.append(f"   {live}/{len(boards)} boards", style="dim")
    title.append(f"   {time.strftime('%H:%M:%S', time.localtime(now))}", style="dim")
    root["head"].update(Panel(title, border_style=wcol if worst is not None else "blue", padding=(0, 1)))

    root["top"].split_row(Layout(boards_panel(m, now), size=56), Layout(links_panel(m, now)))
    root["mid"].split_row(Layout(routes_panel(m), ratio=7), Layout(neighbors_panel(m), ratio=3),
                          Layout(traffic_panel(m), ratio=4))
    root["bot"].split_row(Layout(events_panel(m), ratio=3), Layout(paths_panel(m), ratio=2))
    root["foot"].update(Panel("[bold cyan]Ctrl+C[/] Quit", border_style="blue", padding=(0, 1)))
    return root


# --------------------------------------------------------------------------- main
def main(argv=None):
    ap = argparse.ArgumentParser(description="Terminal dashboard for the ESP-NOW mesh boards.")
    ap.add_argument("--ports", nargs="+", metavar="COM", help="one serial port per board, e.g. COM3 COM4")
    ap.add_argument("--replay", nargs="+", metavar="FILE",
                    help="print one frame from recorded serial captures (one file per board) and exit")
    args = ap.parse_args(argv)
    if hasattr(sys.stdout, "reconfigure"):      # redirected output on Windows defaults to cp1252 (no ●, █, →)
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")

    if args.replay:
        mesh = replay(args.replay)
        console = Console()
        console.print(render(mesh, replay_now(mesh), mode="REPLAY (recorded lines, not live)"),
                      height=max(42, console.size.height))
        return
    if not args.ports:
        ap.error("give the boards' ports, e.g. --ports COM3 COM4 COM5 COM6")

    console = Console()
    mesh = LiveMesh(args.ports)
    mesh.start()
    try:
        with Live(render(mesh, time.time()), console=console, screen=True, auto_refresh=False) as live:
            while True:
                started = time.monotonic()
                mesh.poll()
                live.update(render(mesh, time.time()), refresh=True)
                time.sleep(max(0.0, TICK - (time.monotonic() - started)))
    except KeyboardInterrupt:
        pass
    finally:
        mesh.stop()


if __name__ == "__main__":
    main()

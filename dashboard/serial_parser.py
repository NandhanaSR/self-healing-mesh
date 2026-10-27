"""Parser for the serial output of mesh_node.ino (115200 baud).

Pure Python, no serial or rich imports, so it can be tested without boards.

Formats (from mesh_node.ino, see CLAUDE.md):

  printTables(), every second:
    ---- node %u ----
      nbr %u  %-9s ewma %.1f  slope %.2f  PRR %.2f dPRR %.2f  q %u%% q_pred %.2f
      route -> %u  next %u (cost %.2f) | priority next %u (hops %u)
  events:
    [LINK] node %u: %s -> %s  (ewma %.1f dBm, slope %.2f dB/s, PRR %.2f, q %u%%, q_pred %.2f)
    [FWD] dest %u via %u[ (PRIORITY)]
    [FWD] no route to %u
    [DATA] from %u  s1=%d s2=%d  anomaly=0x%02X/0x%02X[  PRIORITY]
    destination = %u
    priority = ON|OFF
    Node %u ready. Type 0-5 = destination, p = toggle priority.

Values are taken as printed by the board; nothing is recomputed here.
"""
from __future__ import annotations

import re
from dataclasses import dataclass, field
from typing import Optional, Union

LINK_STATES = ("HEALTHY", "DEGRADING", "DEAD")

# --------------------------------------------------------------------------- records


@dataclass(frozen=True)
class NodeHeader:
    node: int                   # MY_ID of the board that printed the block


@dataclass(frozen=True)
class Neighbor:
    nbr: int
    state: str                  # HEALTHY / DEGRADING / DEAD
    ewma: float                 # dBm
    slope: float                # dB/s
    prr: float
    dprr: float
    q: int                      # health, 0-100 %
    q_pred: float               # 0-1


@dataclass(frozen=True)
class Route:
    dest: int
    next: int
    cost: float
    prio_next: int
    hops: int


@dataclass(frozen=True)
class LinkEvent:
    nbr: int                    # the neighbor whose link changed (not the printing board)
    old: str
    new: str
    ewma: float
    slope: float
    prr: float
    q: int
    q_pred: float


@dataclass(frozen=True)
class Forward:
    dest: int
    via: int
    priority: bool


@dataclass(frozen=True)
class NoRoute:
    dest: int


@dataclass(frozen=True)
class DataRx:
    src: int                    # sensor fields are deliberately not kept (no sensors connected)
    priority: bool


@dataclass(frozen=True)
class DestChanged:
    dest: int


@dataclass(frozen=True)
class PriorityChanged:
    on: bool


@dataclass(frozen=True)
class Ready:
    node: int


@dataclass(frozen=True)
class Unknown:
    text: str                   # boot messages, partial lines, anything unrecognised


@dataclass(frozen=True)
class NodeTable:
    """One complete printTables() block: the board's full view at that moment.

    Routes the firmware skipped (unreachable) are simply absent.
    """
    node: int
    neighbors: tuple[Neighbor, ...] = field(default_factory=tuple)
    routes: tuple[Route, ...] = field(default_factory=tuple)


Record = Union[NodeHeader, Neighbor, Route, LinkEvent, Forward, NoRoute, DataRx,
               DestChanged, PriorityChanged, Ready, Unknown]

# --------------------------------------------------------------------------- regexes

_U = r"(\d+)"                                         # %u
_D = r"(-?\d+)"                                       # %d
_F = r"([-+]?(?:\d+(?:\.\d*)?|nan|inf))"              # %.Nf (newlib prints nan / inf)
_X = r"[0-9A-Fa-f]{2,}"                               # %02X
_STATE = r"([A-Z]+)"

_PATTERNS = [
    (NodeHeader, re.compile(rf"---- node {_U} ----")),
    (Neighbor, re.compile(
        rf"nbr {_U}\s+{_STATE}\s+ewma {_F}\s+slope {_F}\s+PRR {_F}\s+dPRR {_F}\s+q {_U}%\s+q_pred {_F}",
        re.I)),
    (Route, re.compile(
        rf"route -> {_U}\s+next {_U} \(cost {_F}\) \| priority next {_U} \(hops {_U}\)", re.I)),
    (LinkEvent, re.compile(
        rf"\[LINK\] node {_U}: {_STATE} -> {_STATE}\s+\(ewma {_F} dBm, slope {_F} dB/s, "
        rf"PRR {_F}, q {_U}%, q_pred {_F}\)", re.I)),
    (Forward, re.compile(rf"\[FWD\] dest {_U} via {_U}( \(PRIORITY\))?")),
    (NoRoute, re.compile(rf"\[FWD\] no route to {_U}")),
    (DataRx, re.compile(rf"\[DATA\] from {_U}\s+s1={_D} s2={_D}\s+anomaly=0x{_X}/0x{_X}(\s+PRIORITY)?")),
    (DestChanged, re.compile(rf"destination = {_U}")),
    (PriorityChanged, re.compile(r"priority = (ON|OFF)")),
    (Ready, re.compile(rf"Node {_U} ready\..*")),
]


def _build(kind, g: tuple) -> Record:
    if kind is NodeHeader:
        return NodeHeader(int(g[0]))
    if kind is Neighbor:
        return Neighbor(int(g[0]), g[1].upper(), float(g[2]), float(g[3]), float(g[4]),
                        float(g[5]), int(g[6]), float(g[7]))
    if kind is Route:
        return Route(int(g[0]), int(g[1]), float(g[2]), int(g[3]), int(g[4]))
    if kind is LinkEvent:
        return LinkEvent(int(g[0]), g[1].upper(), g[2].upper(), float(g[3]), float(g[4]),
                         float(g[5]), int(g[6]), float(g[7]))
    if kind is Forward:
        return Forward(int(g[0]), int(g[1]), g[2] is not None)
    if kind is NoRoute:
        return NoRoute(int(g[0]))
    if kind is DataRx:
        return DataRx(int(g[0]), g[3] is not None)
    if kind is DestChanged:
        return DestChanged(int(g[0]))
    if kind is PriorityChanged:
        return PriorityChanged(g[0] == "ON")
    return Ready(int(g[0]))


def parse_line(line: str) -> Optional[Record]:
    """Parse one serial line. Returns None for a blank line, Unknown if unrecognised."""
    text = line.strip()
    if not text:
        return None
    for kind, rx in _PATTERNS:
        m = rx.fullmatch(text)
        if m:
            return _build(kind, m.groups())
    return Unknown(text)


# --------------------------------------------------------------------------- per-port stream


class BoardStream:
    """Stateful parser for one COM port.

    Groups the nbr/route lines that follow a "---- node N ----" header into a NodeTable.
    The firmware prints no end marker, so a block is emitted when the next non-table line
    (usually the next header, ~1 s later) arrives, or on flush().
    nbr/route lines seen before any header (e.g. port opened mid-block) are dropped,
    because it is unknown which board printed them.
    """

    def __init__(self) -> None:
        self.node_id: Optional[int] = None      # learned from the header or the Ready line
        self._block: Optional[int] = None
        self._nbrs: list[Neighbor] = []
        self._routes: list[Route] = []

    def feed(self, line: str) -> list:
        """Feed one line; returns the records it completes (NodeTable / events / Unknown)."""
        rec = parse_line(line)
        if rec is None:
            return []
        if isinstance(rec, Neighbor):
            if self._block is not None:
                self._nbrs.append(rec)
            return []
        if isinstance(rec, Route):
            if self._block is not None:
                self._routes.append(rec)
            return []

        out = self.flush()
        if isinstance(rec, NodeHeader):
            self.node_id = rec.node
            self._block = rec.node
            return out
        if isinstance(rec, Ready):
            self.node_id = rec.node
        out.append(rec)
        return out

    def flush(self) -> list:
        """Emit the block in progress, if any."""
        if self._block is None:
            return []
        table = NodeTable(self._block, tuple(self._nbrs), tuple(self._routes))
        self._block = None
        self._nbrs, self._routes = [], []
        return [table]

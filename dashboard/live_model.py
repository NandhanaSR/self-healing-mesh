"""Live data model: one serial reader per COM port, merged into what the panels read.

Every value comes from a board's serial output (see serial_parser.py). Nothing here
recomputes link quality or routes; paths are only assembled by following each board's
own next hop.

Check without the dashboard:
    python dashboard/live_model.py COM3 COM4            # live, prints state every second
    python dashboard/live_model.py --replay capture.txt # one capture file per board
"""
from __future__ import annotations

import queue
import sys
import threading
import time
from collections import Counter, deque
from dataclasses import dataclass
from typing import Optional

from serial_parser import (BoardStream, DataRx, DestChanged, Forward, LinkEvent, NodeTable,
                           NoRoute, PriorityChanged, Ready, Unknown)

BAUD = 115200
MAX_NODES = 6               # firmware ids 0..5 (config.h)
PRINT_INTERVAL_S = 1.0      # firmware prints its tables every PRINT_INTERVAL_MS
STALE_S = 3.0               # no table for this long -> board shown as stale
RECONNECT_S = 2.0
HIST_LEN = 14               # samples kept per link for sparklines (one per table, ~1 s)
DEFAULT_DEST_ID = 1         # firmware data destination until a board prints "destination = N"
DATA_INTERVAL_S = 1.0       # each board originates one DATA packet per DATA_INTERVAL_MS
SEND_BURST = 2.0            # own sends allowed back to back (serial lines arrive in chunks)
RELAY_MATCH_S = 2.0         # an upstream "[FWD] .. via B" explains one of B's [FWD] lines this long

LINK_LEVEL = {"HEALTHY": "OK", "DEGRADING": "WARN", "DEAD": "CRIT"}
LINK_ICON = {"HEALTHY": "▲", "DEGRADING": "▼", "DEAD": "✗"}
LEVEL_ICON = {"INFO": "•", "OK": "✓", "WARN": "!", "CRIT": "✗", "ROUTE": "↻"}
MERGE_S = 5.0               # the exact same event again within this long updates its row (×N) instead of adding one


@dataclass(frozen=True)
class PortStatus:
    """Emitted by a SerialReader when its port opens or fails."""
    connected: bool
    detail: str = ""


# --------------------------------------------------------------------------- serial side


def _open_serial(port: str):
    import serial                                   # pyserial; imported lazily so --demo works without it
    ser = serial.Serial()
    ser.port, ser.baudrate, ser.timeout = port, BAUD, 0.2
    # Keep DTR/RTS released: on ESP32 dev boards they drive EN/IO0, so asserting them on
    # open would reset the board and wipe its link history.
    ser.dtr = False
    ser.rts = False
    ser.open()
    return ser


class SerialReader(threading.Thread):
    """Reads one COM port, parses it, and puts (port, time, record) on a shared queue."""

    def __init__(self, port: str, out: queue.Queue, opener=_open_serial):
        super().__init__(name=f"reader-{port}", daemon=True)
        self.port, self.out, self.opener = port, out, opener
        self._stop_event = threading.Event()
        self._ser = None

    def run(self) -> None:
        while not self._stop_event.is_set():
            try:
                self._ser = self.opener(self.port)
            except (OSError, ValueError) as e:          # SerialException is an OSError
                self._emit(PortStatus(False, str(e)))
                self._stop_event.wait(RECONNECT_S)
                continue
            self._emit(PortStatus(True))
            stream, buf = BoardStream(), b""
            try:
                while not self._stop_event.is_set():
                    chunk = self._ser.read(self._ser.in_waiting or 1)
                    if not chunk:
                        continue
                    buf += chunk
                    *lines, buf = buf.split(b"\n")
                    for raw in lines:
                        for rec in stream.feed(raw.decode("utf-8", errors="replace")):
                            self._emit(rec)
            except (OSError, ValueError) as e:
                self._emit(PortStatus(False, str(e) or "port closed"))
            finally:
                self._close()
            self._stop_event.wait(RECONNECT_S)

    def send(self, text: str) -> bool:
        """Write to the board (it accepts '0'-'5' and 'p'). False if the port is not open."""
        ser = self._ser
        if ser is None:
            return False
        try:
            ser.write(text.encode("ascii"))
            return True
        except (OSError, ValueError):
            return False

    def stop(self) -> None:
        self._stop_event.set()

    def _emit(self, rec) -> None:
        self.out.put((self.port, time.time(), rec))

    def _close(self) -> None:
        ser, self._ser = self._ser, None
        try:
            if ser is not None:
                ser.close()
        except (OSError, ValueError):
            pass


# --------------------------------------------------------------------------- model


class Board:
    """What one board (one COM port) has told us, as printed."""

    def __init__(self, port: str):
        self.port = port
        self.node_id: Optional[int] = None
        self.connected = False
        self.error = ""
        self.neighbors = {}                 # nbr id -> serial_parser.Neighbor
        self.routes = {}                    # dest id -> serial_parser.Route
        self.table_time: Optional[float] = None
        self.dest: Optional[int] = None     # unknown until the board prints "destination = N"
        self.priority: Optional[bool] = None
        self.fwd = 0                        # "[FWD] dest .. via .." lines
        self.fwd_prio = 0
        self.no_route = 0                   # "[FWD] no route to .." lines
        self.delivered = 0                  # "[DATA] from .." lines (this board was the destination)
        self.delivered_from = Counter()
        self.ewma_hist = {}                 # nbr id -> deque of ewma (dBm)
        self.qpred_hist = {}                # nbr id -> deque of q_pred
        self._unroutable = set()            # dests currently reporting "no route"

    def stale(self, now: float) -> bool:
        return self.table_time is None or now - self.table_time > STALE_S

    @property
    def label(self) -> str:
        return f"N{self.node_id}" if self.node_id is not None else self.port

    @property
    def data_dest(self) -> int:
        """Where this board's own DATA goes: the last "destination = N" it printed, else the default."""
        return DEFAULT_DEST_ID if self.dest is None else self.dest

    def counting(self, t: float) -> bool:
        """LIVE and identified: only then do its lines count towards delivery."""
        return self.connected and self.node_id is not None and not self.stale(t)


class Delivery:
    """End-to-end DATA delivery for one (sender, destination) pair, counted while both boards are live.

    A relayed packet prints the same "[FWD] dest X" line as an own send, so a sender's [FWD]
    lines towards X are only counted as sent when they are not explained by
      - an "[FWD] dest X via <sender>" from another connected board in the last RELAY_MATCH_S, or
      - being more than one per DATA_INTERVAL_S (own sends are periodic; extra lines are relays).
    """

    def __init__(self, t: float):
        self.sent = 0
        self.delivered = 0
        self.relays = 0                     # [FWD] lines not counted as sent
        self._tokens, self._t = SEND_BURST, t      # start with slack: read times jitter around 1 s
        self._upstream = deque()            # times of "[FWD] dest X via <sender>" from other boards

    def on_upstream(self, t: float) -> None:
        self._upstream.append(t)

    def on_send(self, t: float) -> None:
        while self._upstream and t - self._upstream[0] > RELAY_MATCH_S:
            self._upstream.popleft()
        self._tokens = min(SEND_BURST, self._tokens + max(0.0, t - self._t) / DATA_INTERVAL_S)
        self._t = t
        if self._upstream:
            self._upstream.popleft()
            self.relays += 1
        elif self._tokens >= 1.0 - 1e-6:
            self._tokens -= 1.0
            self.sent += 1
        else:
            self.relays += 1

    @property
    def pct(self) -> Optional[float]:
        return 100.0 * self.delivered / self.sent if self.sent else None


@dataclass(eq=False)
class Event:
    """One row of the event log: who it is about, a short summary, and dim detail that may be cut off."""
    t: float
    level: str                  # INFO / OK / WARN / CRIT / ROUTE
    who: str                    # "N2", "N2→N1" (link as measured by N2), "N2 →N1" (route to N1), "COM3"
    text: str
    detail: str = ""
    icon: str = "•"
    key: tuple = ()             # events with the same key within MERGE_S share a row
    group: tuple = ()           # a different event of the same group in between stops the merge
    count: int = 1
    seq: int = 0                # increases with every new or updated row
    link: Optional[LinkEvent] = None    # the [LINK] line, so the panel can colour its values

    @property
    def stamp(self) -> str:
        return time.strftime("%H:%M:%S", time.localtime(self.t))

    @property
    def msg(self) -> str:
        return " ".join(s for s in (self.who, self.text, self.detail and f"({self.detail})") if s)

    @property
    def important(self) -> bool:
        return self.level != "INFO"


class LiveMesh:
    def __init__(self, ports=()):
        self.queue: queue.Queue = queue.Queue()
        self.boards = {p: Board(p) for p in ports}
        self.readers = {}
        self.events = deque(maxlen=200)     # Event, oldest first
        self.event_seq = 0
        self.link_changes = 0               # [LINK] lines
        self.route_changes = 0              # next / priority next changes seen between snapshots
        self.delivery = {}                  # (sender id, dest id) -> Delivery

    # ---- readers
    def start(self, opener=_open_serial) -> None:
        for port in self.boards:
            r = SerialReader(port, self.queue, opener)
            self.readers[port] = r
            r.start()

    def stop(self) -> None:
        for r in self.readers.values():
            r.stop()

    def send(self, port: str, text: str) -> bool:
        r = self.readers.get(port)
        return r.send(text) if r else False

    # ---- updates
    def poll(self) -> None:
        """Apply everything the readers have queued. Call from the UI thread."""
        while True:
            try:
                port, t, rec = self.queue.get_nowait()
            except queue.Empty:
                return
            self.apply(port, t, rec)

    def log(self, t: float, level: str, who: str, text: str, detail: str = "", icon: str = "",
            key: tuple = (), group: tuple = (), link: Optional[LinkEvent] = None) -> Event:
        """Add an event, or update the row of the same event (same key, default: same words) seen
        within MERGE_S: it moves to the top with the new values and its count goes up.
        With a group, only the newest event of that group can be merged into, so
        H→D, D→H, H→D stays three rows."""
        key = key or (level, who, text)
        self.event_seq += 1
        for e in reversed(self.events):
            if t - e.t > MERGE_S:
                break
            if group and e.group == group and e.key != key:
                break
            if e.key == key:
                self.events.remove(e)
                e.t, e.level, e.text, e.detail, e.icon = t, level, text, detail, icon or LEVEL_ICON.get(level, "•")
                e.count += 1
                e.seq, e.link = self.event_seq, link
                self.events.append(e)
                return e
        e = Event(t, level, who, text, detail, icon or LEVEL_ICON.get(level, "•"), key, group, seq=self.event_seq,
                  link=link)
        self.events.append(e)
        return e

    def apply(self, port: str, t: float, rec) -> None:
        b = self.boards.get(port)
        if b is None:
            b = self.boards[port] = Board(port)

        if isinstance(rec, PortStatus):
            if rec.connected and not b.connected:
                self.log(t, "INFO", port, "open")
            elif not rec.connected and (b.connected or rec.detail != b.error):
                self.log(t, "CRIT", port, "unavailable", rec.detail,
                         key=("port", port, rec.detail), group=("port", port))
            b.connected, b.error = rec.connected, ("" if rec.connected else rec.detail)
        elif isinstance(rec, NodeTable):
            self._set_id(b, rec.node, t)
            self._apply_table(b, rec, t)
        elif isinstance(rec, LinkEvent):
            self.link_changes += 1
            # who = measuring board → neighbor, as in LINK HEALTH (the two directions can differ).
            # Only the exact same transition on the same link merges; every other change keeps its
            # own row (HEALTHY -> DEGRADING -> DEAD is the prediction at work).
            self.log(t, LINK_LEVEL.get(rec.new, "INFO"), f"{b.label}→N{rec.nbr}",
                     f"{rec.old}→{rec.new}  q_pred {rec.q_pred:.2f}",
                     # units (dBm, dB/s) left out so it fits the panel's second line at 140 columns
                     f"ewma {rec.ewma:.1f}  slope {rec.slope:.2f}  PRR {rec.prr:.2f}  q {rec.q}%",
                     icon=LINK_ICON.get(rec.new, "•"),
                     key=("link", port, rec.nbr, rec.old, rec.new), group=("link", port, rec.nbr), link=rec)
        elif isinstance(rec, Forward):
            b.fwd += 1
            b.fwd_prio += rec.priority
            if rec.dest in b._unroutable:
                b._unroutable.discard(rec.dest)
                self.log(t, "OK", f"{b.label} →N{rec.dest}", f"forwarding again via N{rec.via}")
            self._count_send(b, rec.dest, t)
            relay = self.by_id().get(rec.via)
            if b.node_id is not None and relay is not None and relay is not b and relay.data_dest == rec.dest:
                d = self._delivery(relay, t)
                if d is not None:
                    d.on_upstream(t)
        elif isinstance(rec, NoRoute):
            b.no_route += 1
            self._count_send(b, rec.dest, t)
            if rec.dest not in b._unroutable:       # log the start only; the board repeats it every send
                b._unroutable.add(rec.dest)
                self.log(t, "CRIT", f"{b.label} →N{rec.dest}", "no route, data not sent")
        elif isinstance(rec, DataRx):
            if not b.delivered_from[rec.src]:
                self.log(t, "INFO", b.label, f"receiving data from N{rec.src}")
            b.delivered += 1
            b.delivered_from[rec.src] += 1
            sender = self.by_id().get(rec.src)
            if sender is not None and sender.data_dest == b.node_id:
                d = self._delivery(sender, t)
                if d is not None:
                    d.delivered += 1
        elif isinstance(rec, DestChanged):
            b.dest = rec.dest
            self.log(t, "INFO", b.label, f"destination = N{rec.dest}", group=("dest", port))
        elif isinstance(rec, PriorityChanged):
            b.priority = rec.on
            self.log(t, "INFO", b.label, f"priority {'ON' if rec.on else 'OFF'}", group=("prio", port))
        elif isinstance(rec, Ready):
            self._set_id(b, rec.node, t)
            self._reset(b)
            self.log(t, "INFO", b.label, "booted", f"on {port}")
        elif isinstance(rec, Unknown):
            if "FAIL" in rec.text:
                self.log(t, "CRIT", b.label, rec.text)

    def _delivery(self, sender: Board, t: float) -> Optional[Delivery]:
        """The pair for sender -> its data destination, or None while either board is not live."""
        dest = self.by_id().get(sender.data_dest)
        if dest is None or dest is sender or not (sender.counting(t) and dest.counting(t)):
            return None
        key = (sender.node_id, dest.node_id)
        if key not in self.delivery:
            self.delivery[key] = Delivery(t)
        return self.delivery[key]

    def _count_send(self, b: Board, dest: int, t: float) -> None:
        if dest == b.data_dest:
            d = self._delivery(b, t)
            if d is not None:
                d.on_send(t)

    def _set_id(self, b: Board, node: int, t: float) -> None:
        if b.node_id == node:
            return
        b.node_id = node
        for other in self.boards.values():
            if other is not b and other.node_id == node:
                self.log(t, "WARN", f"{b.port}+{other.port}", f"both report MY_ID {node}", "check config.h")

    def _reset(self, b: Board) -> None:
        """Board rebooted: its tables and settings start over (dest/priority back to firmware defaults
        DEFAULT_DEST_ID / OFF, but we only show them once the board prints them)."""
        b.neighbors, b.routes, b.table_time = {}, {}, None
        b.dest = b.priority = None
        b.ewma_hist, b.qpred_hist = {}, {}
        b._unroutable = set()

    def _apply_table(self, b: Board, tab: NodeTable, t: float) -> None:
        new_routes = {r.dest: r for r in tab.routes}
        if b.table_time is not None:            # compare with the previous snapshot from this board
            for d in sorted(set(b.routes) | set(new_routes)):
                old, new = b.routes.get(d), new_routes.get(d)
                if old and new and (old.next, old.prio_next) == (new.next, new.prio_next):
                    continue
                self.route_changes += 1
                who = f"{b.label} →N{d}"
                if old and not new:
                    self.log(t, "CRIT", who, "route lost", f"was via N{old.next}", group=("route", b.port, d))
                    continue
                if not old:
                    text = f"new route via N{new.next}"
                else:                           # only show the hops that changed
                    text = "  ".join(f"{name} N{a}⇒N{z}" for name, a, z in
                                     (("next", old.next, new.next), ("prio", old.prio_next, new.prio_next))
                                     if a != z)
                self.log(t, "ROUTE", who, text, f"cost {new.cost:.2f} · prio next N{new.prio_next}",
                         key=("route", b.port, d, text), group=("route", b.port, d))
        b.routes = new_routes
        b.neighbors = {n.nbr: n for n in tab.neighbors}
        b.table_time = t
        for n in tab.neighbors:
            b.ewma_hist.setdefault(n.nbr, deque(maxlen=HIST_LEN)).append(n.ewma)
            b.qpred_hist.setdefault(n.nbr, deque(maxlen=HIST_LEN)).append(n.q_pred)

    # ---- queries for the panels
    def by_id(self) -> dict:
        return {b.node_id: b for b in self.boards.values() if b.node_id is not None}

    def path(self, src: int, dest: int, priority: bool = False):
        """Follow next hops (or priority next hops) from src to dest across the boards' tables.

        Returns (nodes, status), status one of:
          "ok"        reached dest
          "no-route"  the last node in `nodes` has no route to dest
          "unknown"   the last node in `nodes` is not connected / has printed no table yet
          "loop"      a node repeated (transient while distance-vector converges)
        """
        boards = self.by_id()
        nodes = [src]
        while nodes[-1] != dest:
            b = boards.get(nodes[-1])
            if b is None or b.table_time is None:
                return nodes, "unknown"
            r = b.routes.get(dest)
            if r is None:
                return nodes, "no-route"
            nxt = r.prio_next if priority else r.next
            looped = nxt in nodes
            nodes.append(nxt)
            if looped or len(nodes) > MAX_NODES:
                return nodes, "loop"
        return nodes, "ok"


# --------------------------------------------------------------------------- replay / text check mode


def replay(paths) -> LiveMesh:
    """Feed capture files (one per board) into a LiveMesh, one table per second as the firmware
    prints them. Returns the mesh in its final state."""
    mesh = LiveMesh(paths)
    t0 = time.time()
    for path in paths:
        mesh.boards[path].connected = True      # a recorded board was connected while recording
        stream, t = BoardStream(), t0
        with open(path, encoding="utf-8", errors="replace") as f:
            lines = f.read().splitlines()
        for line in lines:
            if line.startswith("---- node"):
                t += PRINT_INTERVAL_S
            for rec in stream.feed(line):
                mesh.apply(path, t, rec)
        for rec in stream.flush():
            mesh.apply(path, t, rec)
    return mesh


def replay_now(mesh: LiveMesh) -> float:
    """The 'current time' for a replayed mesh: its newest table, so nothing shows as stale."""
    return max((b.table_time or 0.0) for b in mesh.boards.values())



def summary(mesh: LiveMesh, now: float) -> str:
    out = []
    for b in mesh.boards.values():
        if not b.connected and b.table_time is None:
            out.append(f"{b.port}: {'not connected' + (' - ' + b.error if b.error else '')}")
            continue
        age = "no table yet" if b.table_time is None else f"table {now - b.table_time:.1f}s ago"
        flag = "  STALE" if b.stale(now) else ""
        dest = "?" if b.dest is None else b.dest
        prio = "?" if b.priority is None else ("ON" if b.priority else "OFF")
        out.append(f"{b.port}: {b.label}  {age}{flag}  dest={dest} prio={prio}  "
                   f"fwd={b.fwd} no_route={b.no_route} delivered={b.delivered}")
        for n in b.neighbors.values():
            out.append(f"   nbr N{n.nbr} {n.state:<9} ewma {n.ewma:6.1f}  slope {n.slope:5.2f}  "
                       f"PRR {n.prr:.2f} dPRR {n.dprr:5.2f}  q {n.q:3d}%  q_pred {n.q_pred:.2f}")
        for r in b.routes.values():
            nodes, status = mesh.path(b.node_id, r.dest)
            out.append(f"   route →N{r.dest} next N{r.next} cost {r.cost:.2f} | prio next N{r.prio_next} "
                       f"hops {r.hops}   path {'→'.join(f'N{x}' for x in nodes)} [{status}]")
    out.append("-- events --")
    out.extend(f"{e.stamp} {e.level:<5} {e.msg}" + (f"  ×{e.count}" if e.count > 1 else "")
               for e in list(mesh.events)[-10:])
    return "\n".join(out)


def main(argv) -> int:
    import argparse
    ap = argparse.ArgumentParser(description="Text check of the live data model.")
    ap.add_argument("ports", nargs="*", help="COM ports, e.g. COM3 COM4")
    ap.add_argument("--replay", nargs="+", metavar="FILE", help="feed capture files instead of ports")
    args = ap.parse_args(argv)
    if hasattr(sys.stdout, "reconfigure"):      # Windows pipes default to cp1252, which has no "→"
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")

    if args.replay:
        mesh = replay(args.replay)
        print(summary(mesh, replay_now(mesh)))
        return 0

    if not args.ports:
        ap.error("give COM ports or --replay FILE")
    mesh = LiveMesh(args.ports)
    mesh.start()
    try:
        while True:
            time.sleep(PRINT_INTERVAL_S)
            mesh.poll()
            print("\n" + "=" * 100 + "\n" + summary(mesh, time.time()), flush=True)
    except KeyboardInterrupt:
        mesh.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))

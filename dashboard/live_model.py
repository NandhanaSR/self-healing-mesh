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

LINK_LEVEL = {"HEALTHY": "OK", "DEGRADING": "WARN", "DEAD": "CRIT"}


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
        self._stop = threading.Event()
        self._ser = None

    def run(self) -> None:
        while not self._stop.is_set():
            try:
                self._ser = self.opener(self.port)
            except (OSError, ValueError) as e:          # SerialException is an OSError
                self._emit(PortStatus(False, str(e)))
                self._stop.wait(RECONNECT_S)
                continue
            self._emit(PortStatus(True))
            stream, buf = BoardStream(), b""
            try:
                while not self._stop.is_set():
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
            self._stop.wait(RECONNECT_S)

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
        self._stop.set()

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


class LiveMesh:
    def __init__(self, ports=()):
        self.queue: queue.Queue = queue.Queue()
        self.boards = {p: Board(p) for p in ports}
        self.readers = {}
        self.events = deque(maxlen=200)     # (HH:MM:SS, level, msg), same shape as the simulator
        self.link_changes = 0               # [LINK] lines
        self.route_changes = 0              # next / priority next changes seen between snapshots

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

    def log(self, t: float, level: str, msg: str) -> None:
        self.events.append((time.strftime("%H:%M:%S", time.localtime(t)), level, msg))

    def apply(self, port: str, t: float, rec) -> None:
        b = self.boards.get(port)
        if b is None:
            b = self.boards[port] = Board(port)

        if isinstance(rec, PortStatus):
            if rec.connected and not b.connected:
                self.log(t, "INFO", f"{port} open")
            elif not rec.connected and (b.connected or rec.detail != b.error):
                self.log(t, "CRIT", f"{port} unavailable: {rec.detail}")
            b.connected, b.error = rec.connected, ("" if rec.connected else rec.detail)
        elif isinstance(rec, NodeTable):
            self._set_id(b, rec.node, t)
            self._apply_table(b, rec, t)
        elif isinstance(rec, LinkEvent):
            self.link_changes += 1
            self.log(t, LINK_LEVEL.get(rec.new, "INFO"),
                     f"{b.label}↔N{rec.nbr} {rec.old} → {rec.new}  (ewma {rec.ewma:.1f} dBm · "
                     f"slope {rec.slope:.2f} dB/s · PRR {rec.prr:.2f} · q {rec.q}% · q_pred {rec.q_pred:.2f})")
        elif isinstance(rec, Forward):
            b.fwd += 1
            b.fwd_prio += rec.priority
            if rec.dest in b._unroutable:
                b._unroutable.discard(rec.dest)
                self.log(t, "OK", f"{b.label} forwarding to N{rec.dest} again via N{rec.via}")
        elif isinstance(rec, NoRoute):
            b.no_route += 1
            if rec.dest not in b._unroutable:       # log the start only; the board repeats it every send
                b._unroutable.add(rec.dest)
                self.log(t, "CRIT", f"{b.label} has no route to N{rec.dest}")
        elif isinstance(rec, DataRx):
            if not b.delivered_from[rec.src]:
                self.log(t, "INFO", f"{b.label} receiving data from N{rec.src}")
            b.delivered += 1
            b.delivered_from[rec.src] += 1
        elif isinstance(rec, DestChanged):
            b.dest = rec.dest
            self.log(t, "INFO", f"{b.label} data destination = N{rec.dest}")
        elif isinstance(rec, PriorityChanged):
            b.priority = rec.on
            self.log(t, "INFO", f"{b.label} priority {'ON' if rec.on else 'OFF'}")
        elif isinstance(rec, Ready):
            self._set_id(b, rec.node, t)
            self._reset(b)
            self.log(t, "INFO", f"{b.label} on {port} booted")
        elif isinstance(rec, Unknown):
            if "FAIL" in rec.text:
                self.log(t, "CRIT", f"{b.label}: {rec.text}")

    def _set_id(self, b: Board, node: int, t: float) -> None:
        if b.node_id == node:
            return
        b.node_id = node
        for other in self.boards.values():
            if other is not b and other.node_id == node:
                self.log(t, "WARN", f"{b.port} and {other.port} both report MY_ID {node} (check config.h)")

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
                if old and not new:
                    self.log(t, "CRIT", f"{b.label} route →N{d} lost (was next N{old.next})")
                elif new and not old:
                    self.log(t, "ROUTE", f"{b.label} route →N{d} via N{new.next} "
                                         f"(cost {new.cost:.2f}, priority next N{new.prio_next})")
                else:
                    self.log(t, "ROUTE", f"{b.label} route →N{d}: next N{old.next} ⇒ N{new.next}, "
                                         f"priority next N{old.prio_next} ⇒ N{new.prio_next} "
                                         f"(cost {new.cost:.2f})")
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
    out.extend(f"{ts} {lvl:<5} {msg}" for ts, lvl, msg in list(mesh.events)[-10:])
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

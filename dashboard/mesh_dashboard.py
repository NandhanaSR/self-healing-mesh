#!/usr/bin/env python3
"""meshctl - live terminal dashboard for the predictive self-healing ESP-NOW mesh.

Reads each board's serial output (one COM port per board) and shows what the boards
report: link state, ewma, slope, PRR/dPRR, q and q_pred per link, and each board's
next hop / priority next hop per destination. Nothing is recomputed in Python.
The anomaly panel is the exception: no sensors are connected, so it shows generated
data and is labelled DEMO.

Run:   pip install rich pyserial
       python dashboard/mesh_dashboard.py --ports COM3 COM4 COM5 COM6
       python dashboard/mesh_dashboard.py --demo                  (simulated boards, SIM on screen)
       python dashboard/mesh_dashboard.py --replay FILE [FILE...] (one frame from captures)
       add --hide-delivery to any of these to hide the Delivery % figure in TRAFFIC
Keys:  Tab / [ ]  select board     0-5  send destination     p  send priority toggle
       space  pause event log      f  event log: all / important only
       q or Ctrl+C  quit           (0-5 / p go to the selected board only; see keyboard.py)
Best viewed in Windows Terminal of at least 140x42.
"""
from __future__ import annotations

import argparse
import sys
import time

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

from demo_anomaly import HIST_LEN, DemoAnomalies, flag_names
from demo_sim import SimMesh
from keyboard import Keyboard, KeyControl
from live_model import MAX_NODES, LiveMesh, replay, replay_now

# --------------------------------------------------------------------------- config
TICK = 0.25                 # seconds per screen update
SIM_MODE = "SIM (simulated boards, not real)"
REPLAY_MODE = "REPLAY (recorded lines, not live)"
BLOCKS = "▁▂▃▄▅▆▇█"


def spark(vals, lo, hi, width=14):
    out = ""
    for v in list(vals)[-width:]:
        frac = max(0.0, min(1.0, (v - lo) / (hi - lo)))
        out += BLOCKS[int(frac * 7)]
    return out


# --------------------------------------------------------------------------- rendering (live boards)
# Every number below is as printed by a board (via live_model.LiveMesh); nothing is recomputed.
Q_TH = 0.50                 # config.h: q_pred below this -> DEGRADING (used for colour only)
Q_RECOVER = 0.65            # config.h: back to HEALTHY above this
RSSI_MIN, RSSI_MAX = -85, -40   # config.h: RSSI score range, used to scale the ewma sparkline
DEFAULT_DEST_ID = 1         # config.h: data destination a board starts with
DELIVERY_OK, DELIVERY_WARN = 85, 60     # Delivery % colour: green at/above OK, yellow at/above WARN, else red

LINK_RANK = {"HEALTHY": 0, "DEGRADING": 1, "DEAD": 2}
LINK_STYLE = {"HEALTHY": "green", "DEGRADING": "yellow", "DEAD": "bold white on red"}
LINK_LETTER = {"HEALTHY": "H", "DEGRADING": "D", "DEAD": "X"}
LEVEL_STYLE = {"INFO": "cyan", "OK": "green", "WARN": "yellow", "CRIT": "bold red", "ROUTE": "bright_blue"}


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


def boards_panel(m, now, selected=None):
    t = Table(box=box.SIMPLE_HEAVY, expand=True, header_style="bold bright_cyan", pad_edge=False, padding=(0, 1, 0, 0), show_edge=False)
    for col, kw in (("NODE", dict(width=4)), ("PORT", dict(min_width=5)), ("STATUS", dict(width=9)),
                    ("DEST", dict(justify="right", width=4)), ("PRIO", dict(justify="right", width=4)),
                    ("FWD", dict(justify="right")), ("NO RT", dict(justify="right", width=5)),
                    ("DATA", dict(justify="right"))):
        t.add_column(col, **kw)
    for b in sorted_boards(m):
        node = f"N{b.node_id}" if b.node_id is not None else "?"
        if b.port == selected:
            node = f"[bold reverse]▶{node}[/]"
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


def delivery_style(pct):
    return "green" if pct >= DELIVERY_OK else "yellow" if pct >= DELIVERY_WARN else "bold red"


def delivery_rows(m, now):
    """One (label, value) per identified sender: its DATA delivered / sent to its current destination."""
    boards = m.by_id()
    rows = []
    for b in sorted_boards(m):
        if b.node_id is None or b.data_dest == b.node_id:
            continue
        label = f"{b.label} → N{b.data_dest}"
        dest = boards.get(b.data_dest)
        if dest is None:
            rows.append((label, "[dim]dest not connected[/]"))
            continue
        d = m.delivery.get((b.node_id, b.data_dest))
        if d is None or not d.sent:
            rows.append((label, "[dim]—[/]"))
            continue
        pct = min(100.0, d.pct)
        live = b.counting(now) and dest.counting(now)
        style = delivery_style(pct) if live else "dim"
        rows.append((label, f"[{style}]{d.delivered}/{d.sent} ({pct:.0f}%)[/]" + ("" if live else " [dim]paused[/]")))
    return rows


def traffic_panel(m, now, show_delivery=True):
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
    if show_delivery:
        rows = delivery_rows(m, now)
        if rows:
            t.add_row("[bold]Delivery %[/]", "[dim]recv/sent[/]")
            for label, value in rows:
                t.add_row(f"  {label}", value)
    return Panel(t, title="[bold]TRAFFIC[/]  [dim]serial lines since start[/]", border_style="magenta")


EVENT_WHO_W = 11            # widest "who" shown in the event log before it is cut with …


def link_event_text(rec):
    """First row of a link change: states coloured as in LINK HEALTH, q_pred by threshold."""
    out = Text()
    out.append(rec.old, style=LINK_STYLE.get(rec.old, "bright_white"))
    out.append("→", style="bright_white")
    out.append(rec.new, style=LINK_STYLE.get(rec.new, "bright_white"))
    out.append("  q_pred ", style="bright_white")
    out.append(f"{rec.q_pred:.2f}", style=q_style(rec.q_pred))
    return out


def link_values_text(rec):
    """Second row of a link change: dim labels, values coloured as in LINK HEALTH."""
    out = Text(" " * 9)
    for label, value, style in (("ewma", f"{rec.ewma:.1f}", "bright_white"),
                                ("slope", f"{rec.slope:.2f}", "red" if rec.slope < 0 else "bright_white"),
                                ("PRR", f"{rec.prr:.2f}", "bright_white"),
                                ("q", f"{rec.q}%", q_style(rec.q / 100.0))):
        out.append(("  " if label != "ewma" else "") + label + " ", style="dim")
        out.append(value, style=style)
    return out


def events_panel(m, keys=None):
    """One row per event: time, icon, who, summary + dim detail (cut with … rather than wrapped).
    Link changes get a second row with the values the board printed at that moment."""
    paused = keys is not None and keys.paused is not None
    important = keys is not None and keys.important_only
    rows = keys.paused if paused else m.events
    t = Table(box=None, expand=True, show_header=False, padding=(0, 0))
    t.add_column(ratio=1, no_wrap=True, overflow="ellipsis")
    t.add_column(justify="right", no_wrap=True, style="bold")
    shown = [e for e in reversed(rows) if e.important or not important][:40]
    who_w = min(EVENT_WHO_W, max((len(e.who) for e in shown), default=0))
    for e in shown:
        style = LEVEL_STYLE.get(e.level, "white")
        who = e.who if len(e.who) <= who_w else e.who[:who_w - 1] + "…"
        line = Text()
        line.append(f"{e.stamp} ", style="dim")            # dim the time only, not the whole row
        line.append(e.icon, style=style)
        line.append(f" {who:<{who_w}} ", style="bold")
        if e.link is not None:
            line.append_text(link_event_text(e.link))
        else:
            line.append(e.text, style="bold red" if e.level == "CRIT" else "bright_white")
            if e.detail:
                line.append("  " + e.detail, style="dim")
        t.add_row(line, f" ×{e.count}" if e.count > 1 else "")
        if e.link is not None:              # values row, indented under the icon: as much room as possible
            t.add_row(link_values_text(e.link), "")
    title = "[bold]EVENT LOG[/]  [dim]newest first · " + ("important" if important else "all") + "[/]"
    if paused:
        title += f"  [bold black on yellow] PAUSED · {keys.new_since_pause()} new [/]"
    body = t if shown else Text("no important events yet" if important else "no events yet", style="dim")
    return Panel(body, title=title, border_style="yellow" if paused else "cyan")


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


def anomaly_panel(m, anomalies):
    t = Table(box=None, show_header=False, expand=True, padding=(0, 1, 0, 0))
    t.add_column(style="bold", no_wrap=True)
    t.add_column(no_wrap=True)
    t.add_column(justify="right", style="dim")
    t.add_column(no_wrap=True)
    totals = {}
    for b in sorted_boards(m):
        pair = anomalies.sensors.get(b.node_id) if anomalies and b.node_id is not None else None
        if not pair:
            continue
        for i, s in enumerate(pair, 1):
            names = flag_names(s.shown_flags)
            flag = Text(" ".join(names), style="bold red") if names else Text("ok", style="dim green")
            label = f"{b.label} s{i}" if i == 1 else " " * len(b.label) + f" s{i}"
            t.add_row(label, Text(spark(s.hist, 0, 4095, width=8), style="yellow"), str(s.value), flag)
            for name, c in s.counts.items():
                totals[name] = totals.get(name, 0) + c
    body = [t] if t.row_count else [Text("waiting for boards…", style="dim")]
    if totals:
        body.append(Text("flagged: " + " · ".join(f"{k} {v}" for k, v in totals.items()), style="dim"))
    return Panel(Group(*body), title="[bold]ANOMALY[/] [bold black on yellow] DEMO [/]",
                 subtitle="[dim]generated · no sensors[/]", border_style="yellow")


def footer_text(mode, keys):
    if mode == REPLAY_MODE:
        return "[dim]replay: one recorded frame, no keys[/]"
    if keys is None:
        return "[dim]keyboard off (input is not a terminal)[/] · [bold cyan]Ctrl+C[/] Quit"
    return (f"keys → [bold reverse] {keys.target_label()} [/]  [bold cyan]Tab[/]/[bold cyan]\\[ ][/] board · "
            f"[bold cyan]0-5[/] dest · [bold cyan]p[/] prio · [bold cyan]space[/] pause log · "
            f"[bold cyan]f[/] filter log · [bold cyan]q[/] quit")


def render(m, now, mode="LIVE", anomalies=None, keys=None, show_delivery=True):
    """keys: the KeyControl when the keyboard is active, else None."""
    root = Layout()
    root.split_column(Layout(name="head", size=3), Layout(name="top", ratio=10),
                      Layout(name="mid", ratio=9), Layout(name="bot", ratio=9), Layout(name="foot", size=3))
    boards = list(m.boards.values())
    live = sum(1 for b in boards if b.table_time is not None) if mode == REPLAY_MODE else \
        sum(1 for b in boards if b.connected and not b.stale(now))
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

    root["top"].split_row(Layout(boards_panel(m, now, keys.selected if keys else None), size=56), Layout(links_panel(m, now)))
    root["mid"].split_row(Layout(routes_panel(m), ratio=7), Layout(neighbors_panel(m), ratio=3),
                          Layout(traffic_panel(m, now, show_delivery), ratio=4))
    root["bot"].split_row(Layout(events_panel(m, keys), ratio=5), Layout(paths_panel(m), ratio=4),
                          Layout(anomaly_panel(m, anomalies), ratio=3))
    root["foot"].update(Panel(footer_text(mode, keys), border_style="blue", padding=(0, 1)))
    return root


# --------------------------------------------------------------------------- main
def run(mesh, mode, show_delivery=True):
    """Screen loop: read keys, drain the readers, step the DEMO sensors once a second, redraw."""
    console = Console()
    anomalies = DemoAnomalies()
    next_sample = 0.0
    try:
        with Keyboard() as kb, Live(console=console, screen=True, auto_refresh=False) as live:
            keys = KeyControl(mesh) if kb.enabled else None
            while True:
                started = time.monotonic()
                for key in kb.read():
                    if not keys.handle(key, time.time()):
                        return
                mesh.poll()
                if started >= next_sample:
                    anomalies.step(mesh.by_id())
                    next_sample = started + 1.0
                live.update(render(mesh, time.time(), mode, anomalies, keys, show_delivery), refresh=True)
                time.sleep(max(0.0, TICK - (time.monotonic() - started)))
    except KeyboardInterrupt:
        pass
    finally:
        mesh.stop()


def main(argv=None):
    ap = argparse.ArgumentParser(description="Terminal dashboard for the ESP-NOW mesh boards.")
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--ports", nargs="+", metavar="COM", help="one serial port per board, e.g. COM3 COM4")
    src.add_argument("--demo", action="store_true", help="simulated boards (SIM on screen), no hardware needed")
    src.add_argument("--replay", nargs="+", metavar="FILE",
                     help="print one frame from recorded serial captures (one file per board) and exit")
    ap.add_argument("--hide-delivery", action="store_true", help="hide the Delivery %% figure in TRAFFIC")
    args = ap.parse_args(argv)
    show_delivery = not args.hide_delivery
    if hasattr(sys.stdout, "reconfigure"):      # redirected output on Windows defaults to cp1252 (no ●, █, →)
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")

    if args.replay:
        mesh = replay(args.replay)
        anomalies = DemoAnomalies()
        for _ in range(HIST_LEN):
            anomalies.step(mesh.by_id())
        console = Console()
        console.print(render(mesh, replay_now(mesh), REPLAY_MODE, anomalies, show_delivery=show_delivery),
                      height=max(42, console.size.height))
    elif args.demo:
        sim = SimMesh()
        mesh = LiveMesh(sim.ports)
        mesh.start(opener=sim.open)             # same SerialReader path as real ports
        run(mesh, SIM_MODE, show_delivery)
    else:
        mesh = LiveMesh(args.ports)
        mesh.start()
        run(mesh, "LIVE", show_delivery)


if __name__ == "__main__":
    main()

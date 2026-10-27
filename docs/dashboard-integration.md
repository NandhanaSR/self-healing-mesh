# Dashboard integration plan

Goal: use the existing `rich` terminal dashboard (`mesh_dashboard.py`, built separately for
this project) to show the real boards live, without changing the firmware.

## What the dashboard is today

- It is a **simulator**. The `Mesh` class (fake RF, Dijkstra routing, fake traffic)
  invents every number. It reads no serial, file or socket.
- The rendering half (panels, layout, colours, sparklines, event log) is good and reusable.
- So the work is: replace the fake data source with a live one, then adjust panels to
  match what the firmware actually provides.

## Mismatches and decisions

| Dashboard assumes | Firmware actually has | Decision |
|---|---|---|
| ESP-NOW + LoRa hybrid, handovers, failback | ESP-NOW only | Remove LoRa / handover everywhere |
| Nodes 1-4, N1 = gateway | IDs 0-5, data sink = `DEFAULT_DEST_ID` (1) | Use firmware IDs; call the destination "sink" |
| GOOD / DEGRADED / CRITICAL / DOWN | HEALTHY / DEGRADING / DEAD | Use firmware states |
| Health computed in Python from RSSI + loss | `q` and `q_pred` computed on the board | Display board values, no recomputation |
| No prediction columns | `slope`, `dPRR`, `q_pred` | **Add them** - this is the core of the project |
| Central Dijkstra, full path, seq number | Per-board next hop, cost, priority next hop, hops | Show next + priority next; build path by following next hops |
| Per-node TX/RX/FWD/DROP, latency | Not printed | Drop; optionally count `[FWD]` / `[DATA]` lines |
| MAC column | Not printed | Drop |
| Anomaly / sensor data | No sensors connected (values are noise) | Dummy data panel, labelled `DEMO` |
| Keys `f`/`g` inject interference | Not possible on real hardware | Keys send `0`-`5` / `p` to a board instead |
| `termios` keyboard input (Linux only) | Developer is on Windows | Support `msvcrt` too |

## Firmware output -> dashboard panel

| Serial line | Panel |
|---|---|
| `---- node N ----` | Starts a block: which board the following lines belong to |
| `  nbr ...` | Link Health, Neighbor table |
| `  route -> ...` | Routing table, Active path view |
| `[LINK] node X: OLD -> NEW (...)` | Event log (state change, with q_pred) |
| `[FWD] dest X via Y` / `[FWD] no route to X` | Event log; forwarded / no-route counts |
| `[DATA] from X ...` | Delivered count at the sink (sensor fields ignored) |

Exact printf formats are in `CLAUDE.md`.

## Steps

1. Put the dashboard in `dashboard/mesh_dashboard.py` (original file, unchanged) and commit.
2. Write a serial parser for the formats above, with tests using recorded/sample lines
   (works without boards).
3. Add a live data model that fills what the panels read, one serial reader per COM port.
4. Rework panels: firmware states, slope / q_pred columns, routing with priority next hop,
   remove LoRa, MAC and packet-counter columns.
5. Add the dummy anomaly panel (`DEMO`) and keep the simulator behind `--demo` (`SIM` on screen).
6. Windows keyboard support; keys send `0`-`5` / `p` to the selected board.
7. Run: `python dashboard/mesh_dashboard.py --ports COM3 COM4 COM5 COM6`.

## Windows setup (no WSL needed)

1. Python from python.org - tick "Add python.exe to PATH".
2. Git for Windows (git-scm.com), defaults.
3. Claude Code: in PowerShell `irm https://claude.ai/install.ps1 | iex`.
4. VS Code + the Claude Code extension; clone the repo, switch to `dashboard-integration`.
5. In the project folder:
   ```powershell
   python -m venv .venv
   .venv\Scripts\activate
   pip install rich pyserial
   ```
   If activation is blocked: `Set-ExecutionPolicy -Scope CurrentUser RemoteSigned`.
6. Use Windows Terminal to run the dashboard (old `cmd` garbles box characters).

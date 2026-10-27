# CLAUDE.md

Predictive self-healing ESP-NOW mesh on ESP32 boards. The point of the project is
**link-quality prediction** (EWMA + slope + delta-PRR -> q_pred) and routing around
links before they die. See README.md for the module layout and formulas.

Current work: integrating a terminal dashboard (`dashboard/`) with the real boards.
Full plan and background: `docs/dashboard-integration.md`.

## Hard rules

1. **The firmware is the source of truth.** Adapt the dashboard to the firmware, never
   the other way round. Do not edit anything in `mesh_node/` unless it is a genuine bug
   in the firmware's logic. If you think you found one, explain it and ask before changing it.
   "The dashboard would be easier if the firmware printed X" is not a reason to change it.
2. **ESP-NOW only.** There is no LoRa hardware. Remove/hide every LoRa, dual-radio,
   handover and failback feature in the dashboard.
3. **No sensors are connected.** Pins 34/35 float, so the `s1`/`s2`/`anomaly` values in
   `[DATA]` lines are noise. The dashboard ignores them; the anomaly panel uses generated
   dummy data and must be clearly labelled `DEMO`.
4. **Everything else on the dashboard is live data from the boards.** Never present
   simulated numbers as real. Keep the original simulator only behind a `--demo` flag,
   with `SIM` shown on screen.

## Firmware facts the dashboard must match

- Node IDs `0..5` (`MAX_NODES 6`). Each board sets `MY_ID` in `config.h`. There is no
  "gateway": data goes to `DEFAULT_DEST_ID` (1), changeable over serial with `0`-`5`.
- Link states: `HEALTHY`, `DEGRADING`, `DEAD` (not GOOD/DEGRADED/CRITICAL/DOWN).
- Link values come from the board: `ewma` (dBm, alpha 0.30), `slope` (dB/s), `PRR`,
  `dPRR`, `q` (health %, 0-100), `q_pred` (0-1). Display them; do not recompute them.
  Thresholds: `Q_TH 0.50` (-> DEGRADING), `Q_RECOVER 0.65` (-> HEALTHY).
- Routing is distance-vector: each board knows only its own `next` hop, `cost`,
  `priority next` hop and `hops` per destination. Full paths are built by following
  next hops across the boards' tables.
- Not available from the firmware: MAC addresses, TX/RX/drop counters, latency.
  Drop those columns rather than adding firmware output.
- ESP32-C3 mini boards run at reduced TX power (`WIFI_POWER_8_5dBm`, set in
  `transport_init()` in `transport.cpp`) because of a hardware antenna flaw: at full power
  they fail and overheat. Their links are weaker than the DevKits', so lower ewma/q on links
  to or from a C3 is expected and is not a bug.

## Serial output (115200 baud), exact formats from `mesh_node.ino`

Printed every second by `printTables()`:
```
---- node %u ----
  nbr %u  %-9s ewma %.1f  slope %.2f  PRR %.2f dPRR %.2f  q %u%% q_pred %.2f
  route -> %u  next %u (cost %.2f) | priority next %u (hops %u)
```
Event lines:
```
[LINK] node %u: %s -> %s  (ewma %.1f dBm, slope %.2f dB/s, PRR %.2f, q %u%%, q_pred %.2f)
[FWD] dest %u via %u            (suffix " (PRIORITY)" for urgent traffic)
[FWD] no route to %u
[DATA] from %u  s1=%d s2=%d  anomaly=0x%02X/0x%02X   (suffix "  PRIORITY"; sensor fields ignored)
destination = %u
priority = ON|OFF
Node %u ready. Type 0-5 = destination, p = toggle priority.
```
Serial input accepted by a board: `0`-`5` (data destination), `p` (toggle priority).

## Environment

- Developer runs **plain Windows** (no WSL): Windows Terminal, Python venv, `pyserial`
  with COM ports (`COM3`, `COM4`, ...). Each board is on its own USB port.
- The dashboard's keyboard input must work on Windows (`msvcrt`) as well as POSIX (`termios`).
- A COM port can be opened by only one program at a time (Arduino Serial Monitor vs dashboard).
- Firmware is built and flashed with Arduino IDE 2.x (ESP32 core 3.x). Laptop tests for the
  firmware logic: `cd host_test && make run` (needs g++/make).

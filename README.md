# Predictive Self-Healing IoT Mesh - modular version

## Layout
mesh_node/            <- the Arduino sketch (open mesh_node.ino in Arduino IDE)
  config.h            every tunable number; MY_ID is the only per-board line
  packets.h           wire format (common header on every packet)
  neighbor_table.*    Module 1  neighbor discovery table
  link_health.*       Module 2  EWMA + least-squares slope + Healthy/Degrading/Dead
  routing.*           Module 3+4 distance-vector routing + priority (fewest-hop) override
  anomaly.*           Module 5  spike / jump / stuck detection (statistics only)
  transport.*         ESP-NOW wrapper (only file that touches ESP-NOW; uses a queue)
  sensors.*           analog reads (swap pots for LDR here only)
  mesh_node.ino       setup() + loop(), glue only
host_test/            laptop tests - NO ESP32 needed
  make run            runs all three tests

## Status
Tested on a laptop (g++): link_health, routing, anomaly.
NOT yet compiled or run on hardware: transport.*, sensors.*, mesh_node.ino
(they need the ESP32 Arduino core 3.x). First job when you get the boards:
compile it, flash 2 boards, check the serial output.

## Per-board steps
1. Arduino IDE 2.x + "esp32 by Espressif Systems" board package, version 3.x
2. Edit MY_ID in config.h (0,1,2,3...) and flash
3. Serial monitor 115200. Type 0-5 to pick destination, p to toggle priority.

## Pins
Sensors use GPIO 34 and 35 (ADC1). Do not use ADC2 pins (e.g. 12, 13) - they stop
working when WiFi/ESP-NOW is on.

## Tuning with real data
Copy RSSI from the serial logs into a CSV (lines: t_ms,rssi) and run:
  cd host_test && make && ./sim_link_health trace.csv
Adjust Q_TH, Q_RECOVER, PREDICT_HORIZON_S in config.h until Degrading fires with enough warning before Dead.

## Formulas: paper vs code (TinyHeal, Miller 2026)
| Paper | In our code (file) | Note |
|---|---|---|
| EWMA smoothing | E = 0.3*RSSI + 0.7*E_old (link_health.cpp) | standard form, cf. RFC 6298 |
| dPRR(t) = PRR(t) - PRR(t-1)  (eq.7) | link_on_tx(): PRR over last 5 tx, dprr = new - old | exact |
| window K = 5  (eq.5) | SLOPE_WINDOW = 5 | exact |
| prediction q_hat(t+tau)  (eq.3/10), Micro-LSTM | linear extrapolation: E + slope*tau and PRR + dPRR, tau = 3 s | NO ML - this is our replacement |
| degradation flag y = 1 if q_hat < Q_th  (eq.4/11) | HEALTHY -> DEGRADING when q_pred < Q_TH (0.5) | exact rule, our predictor |
| routing penalty = COST_SCALE * (1 - q_pred) | link_penalty(): penalty = 10*(1 - q_pred) | predicted quality q_pred directly used for routing penalty |
| hysteresis: switch only if cur_cost - new_cost > 0.10 | routing.cpp: keep current next hop unless new path cheaper by > ROUTE_HYST_MARGIN (0.10f); dead next hop = switch immediately | floating-point cost hysteresis |
| localized update, one-hop advert  (sec 2.3) | triggered route advert on link-state change (mesh_node.ino) | |
Our own (not from any paper): least-squares slope in dB/s, quality weights 0.6/0.4, anomaly detector, priority (fewest-hop) path.

## Ablation (like the paper's E4) - run `make ablation` in host_test
- delta-PRR on/off, "interference" scenario (RSSI stays good, packets start failing): warning before Dead 15 s vs 5 s
- delta-PRR on/off, "dying signal" scenario: no difference (RSSI slope already warns) - say this honestly
- route hysteresis on/off: routing tests show an equal-cost alternative stops causing a switch
These are laptop SIMULATIONS with synthetic traces, not hardware results. Replace with real logs when boards are available.

## Other references
- Routing: RIP-style distance vector with poison reverse (RFC 2453), periodic + event-triggered adverts
- File-per-function layout: modelled on Arregui Almeida et al., Sensors 2025 (LoRa mesh on ESP32)

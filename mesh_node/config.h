// config.h - every tunable number lives here. Nothing else has magic numbers.
#pragma once
#include <stdint.h>

// ---- Per-board setting: the ONLY line that changes between boards ----
#define MY_ID 0 // 0..MAX_NODES-1, unique per board

// ---- Network ----
#define MAX_NODES 6    // node ids are 0..5 (4-5 boards in the demo)
#define WIFI_CHANNEL 1 // all boards must use the same channel
#define MAX_TTL 6
#define NO_ROUTE 0xFF
#define DEFAULT_DEST_ID 1 // where sensor data is sent (change via serial: '0'..'5')

// ---- Timing (ms) ----
#define HELLO_INTERVAL_MS 2000
#define HEARTBEAT_INTERVAL_MS 500
#define ADV_INTERVAL_MS 2000 // periodic route advertisement (RIP-style)
#define DATA_INTERVAL_MS 1000
#define PRINT_INTERVAL_MS 1000
#define DEAD_TIMEOUT_MS 2500 // = 5 missed heartbeats

// ---- Link health: formulas follow TinyHeal (Miller 2026), minus the ML model ----
#define EWMA_ALPHA 0.30f // E_t = a*RSSI + (1-a)*E_(t-1)
#define RSSI_MIN (-85)   // RSSI score 0
#define RSSI_MAX (-40)   // RSSI score 100
#define SLOPE_WINDOW 5   // K = 5 samples (TinyHeal's window), least-squares slope in dB/s
#define W_RSSI 0.6f      // quality q = W_RSSI*rssiScore + W_DELIVERY*PRR
#define W_DELIVERY 0.4f
#define PRR_WINDOW_TX 5        // PRR measured over the last 5 transmissions
#define PREDICT_HORIZON_S 3.0f // tau: look-ahead (TinyHeal: tau <= 3 s works best)
#define Q_TH 0.50f             // eq.4/11: degradation flag y=1 when predicted quality < Q_TH
#define Q_RECOVER 0.65f        // back to Healthy only when predicted quality > this (hysteresis)
#ifndef USE_DELTA_PRR
#define USE_DELTA_PRR 1 // 1 = use delta-PRR trend (eq.7); 0 = ablation "w/o delta-PRR"
#endif

// ---- Routing ----
#define COST_HEALTHY 1.0f
#define COST_PENALTY 5.0f     // default penalty for a Degrading link (comes from q_pred)
#define COST_SCALE 10.0f      // Degrading penalty = COST_SCALE * (1.0f - q_pred)
#define ROUTE_INF 1000000.0f  // "unreachable" (RIP uses 16; we have more headroom)
#ifndef USE_ROUTE_HYSTERESIS
#define USE_ROUTE_HYSTERESIS 1 // 1 = eq.15 switch rule; 0 = ablation "w/o hysteresis"
#endif
#define ROUTE_HYST_MARGIN 0.10f // switch next hop only if new path is cheaper by MORE than this

// ---- Sensors / anomaly detection ----
#define SENSOR1_PIN 34 // ADC1 pins only (ADC2 breaks when WiFi is on)
#define SENSOR2_PIN 35
#define AN_WINDOW 10
#define AN_MIN_SAMPLES 5
#define AN_SPIKE_SIGMA 3.0f
#define AN_MIN_STD 10.0f  // ADC counts; avoids divide-by-zero on flat signals
#define AN_JUMP_DELTA 800 // ADC counts (0..4095) between consecutive samples
#define AN_STUCK_TOL 2    // samples within +-2 counts count as "same"
#define AN_STUCK_COUNT 20 // 20 same samples in a row = stuck sensor

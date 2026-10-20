// link_health.h - Module 2: EWMA + slope + delta-PRR + predicted quality + 3-state model.
// Pure C++ (no Arduino/ESP calls) so it can be tested on a laptop.
//
// Formulas (numbers refer to TinyHeal, Miller 2026, Artif. Intell. Emerg. Technol. 3(1):2):
//   EWMA          E_t = a*RSSI_t + (1-a)*E_(t-1)
//   slope         least-squares over last K=5 EWMA samples, dB/s        (our own; paper has no slope eq.)
//   PRR, dPRR     PRR over a short window; dPRR(t) = PRR(t) - PRR(t-1)  (eq. 7)
//   quality       q = 0.6*rssiScore + 0.4*PRR                           (our own weighting)
//   prediction    q_pred(t+tau) = same formula on  E+slope*tau  and  PRR+dPRR
//                 (linear extrapolation replaces the paper's Micro-LSTM, eq. 3/10)
//   flag          y = 1 if q_pred < Q_TH                                (eq. 4/11)  -> DEGRADING
//   penalty       penalty = COST_SCALE * (1 - q_pred)
#pragma once
#include <stdint.h>
#include "config.h"

enum LinkState : uint8_t
{
  LINK_HEALTHY = 0,
  LINK_DEGRADING = 1,
  LINK_DEAD = 2
};

struct LinkStats
{
  float ewma;  // smoothed RSSI (dBm)
  float slope; // dB per second
  float prr;   // packet reception ratio over the last window (0..1)
  float dprr;  // PRR(t) - PRR(t-1)
  uint8_t winSent, winAcked;
  uint32_t lastSeenMs;
  uint8_t health; // current quality 0..100
  float qPred;    // predicted quality tau seconds ahead (0..1)
  LinkState state;
  uint32_t histT[SLOPE_WINDOW];
  float histY[SLOPE_WINDOW];
  uint8_t histCount, histHead;
};

void link_init(LinkStats &s, int8_t rssi, uint32_t nowMs);
void link_on_rx(LinkStats &s, int8_t rssi, uint32_t nowMs);
void link_on_tx(LinkStats &s, bool success, uint32_t nowMs);
void link_tick(LinkStats &s, uint32_t nowMs);

float link_penalty(const LinkStats &s);

const char *link_state_name(LinkState st);

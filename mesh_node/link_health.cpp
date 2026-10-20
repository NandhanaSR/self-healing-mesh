#include "link_health.h"

static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

static float rssi_score(float rssi) {   // 0..100
  float v = clampf(rssi, RSSI_MIN, RSSI_MAX);
  return (v - RSSI_MIN) * 100.0f / (RSSI_MAX - RSSI_MIN);
}

static void hist_push(LinkStats& s, uint32_t t, float y) {
  s.histT[s.histHead] = t;
  s.histY[s.histHead] = y;
  s.histHead = (s.histHead + 1) % SLOPE_WINDOW;
  if (s.histCount < SLOPE_WINDOW) s.histCount++;
}

// Least-squares slope of ewma vs time, in dB/s:
//   m = (n*sum(t*y) - sum(t)*sum(y)) / (n*sum(t^2) - (sum(t))^2)
static float compute_slope(const LinkStats& s) {
  if (s.histCount < 3) return 0.0f;
  uint8_t n = s.histCount;
  uint8_t start = (s.histHead + SLOPE_WINDOW - n) % SLOPE_WINDOW;
  uint32_t t0 = s.histT[start];
  double sT = 0, sY = 0, sTT = 0, sTY = 0;
  for (uint8_t i = 0; i < n; i++) {
    uint8_t idx = (start + i) % SLOPE_WINDOW;
    double t = (double)(s.histT[idx] - t0) / 1000.0;
    double y = s.histY[idx];
    sT += t; sY += y; sTT += t * t; sTY += t * y;
  }
  double denom = n * sTT - sT * sT;
  if (denom < 1e-9) return 0.0f;
  return (float)((n * sTY - sT * sY) / denom);
}

// current quality and predicted quality (tau seconds ahead), both 0..1
static void recompute_quality(LinkStats& s) {
  float q = (W_RSSI * rssi_score(s.ewma)) / 100.0f + W_DELIVERY * s.prr;
  s.health = (uint8_t)clampf(q * 100.0f, 0.0f, 100.0f);

  float rssiAhead = s.ewma + s.slope * PREDICT_HORIZON_S;                 // linear extrapolation
  float prrAhead  = USE_DELTA_PRR ? clampf(s.prr + s.dprr, 0.0f, 1.0f)    // one window ahead (eq. 7 trend)
                                  : s.prr;
  s.qPred = clampf((W_RSSI * rssi_score(rssiAhead)) / 100.0f + W_DELIVERY * prrAhead, 0.0f, 1.0f);
}

static void update_state(LinkStats& s, uint32_t now) {
  if (now - s.lastSeenMs > DEAD_TIMEOUT_MS) { s.state = LINK_DEAD; return; }
  if (s.state == LINK_HEALTHY) {
    if (s.qPred < Q_TH) s.state = LINK_DEGRADING;          // y = 1 (eq. 4/11): link predicted to fail soon
  } else if (s.state == LINK_DEGRADING) {
    if (s.qPred > Q_RECOVER) s.state = LINK_HEALTHY;       // hysteresis: Q_RECOVER > Q_TH
  }
}

void link_init(LinkStats& s, int8_t rssi, uint32_t nowMs) {
  s = LinkStats();
  s.ewma = rssi;
  s.slope = 0;
  s.prr = 1.0f;
  s.dprr = 0.0f;
  s.lastSeenMs = nowMs;
  s.health = 100;
  s.qPred = 1.0f;
  s.state = LINK_HEALTHY;
  hist_push(s, nowMs, s.ewma);
  recompute_quality(s);
}

void link_on_rx(LinkStats& s, int8_t rssi, uint32_t nowMs) {
  if (s.state == LINK_DEAD) {
    // Came back from the dead: restart measurement, enter probation (Degrading).
    link_init(s, rssi, nowMs);
    s.state = LINK_DEGRADING;
    return;
  }
  s.ewma = EWMA_ALPHA * rssi + (1.0f - EWMA_ALPHA) * s.ewma;
  s.lastSeenMs = nowMs;
  hist_push(s, nowMs, s.ewma);
  s.slope = compute_slope(s);
  recompute_quality(s);
  update_state(s, nowMs);
}

void link_on_tx(LinkStats& s, bool success, uint32_t nowMs) {
  s.winSent++;
  if (success) s.winAcked++;
  if (s.winSent >= PRR_WINDOW_TX) {                         // window full: close it
    float newPrr = (float)s.winAcked / (float)s.winSent;
    s.dprr = newPrr - s.prr;                                // dPRR(t) = PRR(t) - PRR(t-1)   (eq. 7)
    s.prr = newPrr;
    s.winSent = s.winAcked = 0;
  }
  recompute_quality(s);
  update_state(s, nowMs);
}

void link_tick(LinkStats& s, uint32_t nowMs) { update_state(s, nowMs); }

// Routing penalty uses qPred directly: penalty = COST_SCALE * (1.0f - qPred)
float link_penalty(const LinkStats& s) {
  float penalty = COST_SCALE * (1.0f - s.qPred);
  return penalty < 0.0f ? 0.0f : penalty;
}

const char* link_state_name(LinkState st) {
  switch (st) {
    case LINK_HEALTHY:   return "HEALTHY";
    case LINK_DEGRADING: return "DEGRADING";
    case LINK_DEAD:      return "DEAD";
  }
  return "?";
}

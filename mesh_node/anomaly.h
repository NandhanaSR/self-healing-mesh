// anomaly.h - Module 5: spike / jump / stuck-value detection on one analog sensor.
// Statistics only (no ML). Pure C++.
#pragma once
#include <stdint.h>
#include "config.h"
#include "packets.h"   // ANOM_* flags

struct AnomalyDetector {
  int16_t buf[AN_WINDOW];
  uint8_t n, head;
  int16_t prev;
  bool    hasPrev;
  uint8_t sameCount;
};

void    anomaly_init(AnomalyDetector& a);
uint8_t anomaly_update(AnomalyDetector& a, int16_t x);   // returns ANOM_* bitmask (0 = normal)

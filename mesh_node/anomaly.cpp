#include "anomaly.h"
#include <math.h>
#include <string.h>

void anomaly_init(AnomalyDetector& a) { memset(&a, 0, sizeof(a)); }

uint8_t anomaly_update(AnomalyDetector& a, int16_t x) {
  uint8_t flags = 0;

  // SPIKE: far from the recent mean (more than K standard deviations)
  if (a.n >= AN_MIN_SAMPLES) {
    float mean = 0, var = 0;
    for (uint8_t i = 0; i < a.n; i++) mean += a.buf[i];
    mean /= a.n;
    for (uint8_t i = 0; i < a.n; i++) var += (a.buf[i] - mean) * (a.buf[i] - mean);
    float sd = sqrtf(var / a.n);
    if (sd < AN_MIN_STD) sd = AN_MIN_STD;
    if (fabsf(x - mean) > AN_SPIKE_SIGMA * sd) flags |= ANOM_SPIKE;
  }

  if (a.hasPrev) {
    int d = x - a.prev; if (d < 0) d = -d;
    // JUMP: big change between two consecutive samples
    if (d > AN_JUMP_DELTA) flags |= ANOM_JUMP;
    // STUCK: value basically unchanged for many samples in a row
    if (d <= AN_STUCK_TOL) a.sameCount = (a.sameCount < 255) ? a.sameCount + 1 : 255; else a.sameCount = 0;
    if (a.sameCount >= AN_STUCK_COUNT) flags |= ANOM_STUCK;
  }

  a.buf[a.head] = x;
  a.head = (a.head + 1) % AN_WINDOW;
  if (a.n < AN_WINDOW) a.n++;
  a.prev = x; a.hasPrev = true;
  return flags;
}

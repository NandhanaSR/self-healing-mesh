#include <cstdio>
#include <cstdlib>
#include "../mesh_node/anomaly.h"

#define CHECK(cond, msg) do { if (!(cond)) { printf("FAIL: %s\n", msg); exit(1); } else printf("ok   : %s\n", msg); } while (0)

int main() {
  AnomalyDetector a; anomaly_init(a);
  uint8_t f = 0; int v;
  // 30 normal samples: ~2000 with +-4 noise
  for (int i = 0; i < 30; i++) { v = 2000 + (i * 7 % 9) - 4; f |= anomaly_update(a, v); }
  CHECK(f == 0, "normal noisy signal raises no flags");

  uint8_t s = anomaly_update(a, 3600);
  CHECK((s & ANOM_SPIKE) && (s & ANOM_JUMP), "sudden 2000 -> 3600 flagged as SPIKE + JUMP");

  anomaly_init(a);
  uint8_t last = 0;
  for (int i = 0; i < 40; i++) last = anomaly_update(a, 1234);
  CHECK(last & ANOM_STUCK, "40 identical readings flagged STUCK");

  anomaly_init(a);
  for (int i = 0; i < 30; i++) anomaly_update(a, 2000 + (i * 7 % 9) - 4);
  uint8_t j = anomaly_update(a, 2900);
  CHECK(j & ANOM_JUMP, "+900 step flagged JUMP");

  printf("ALL ANOMALY TESTS PASSED\n");
  return 0;
}

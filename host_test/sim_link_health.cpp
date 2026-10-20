// Replays an RSSI trace through link_health.cpp - no ESP32 needed.
//   ./sim_link_health              -> built-in synthetic "link slowly dying" trace
//   ./sim_link_health trace.csv    -> your own trace, lines: t_ms,rssi   (rssi blank/-999 = packet lost)
// When teammates run the boards, copy their serial RSSI log into a CSV and replay it here
// to tune the thresholds in config.h without touching hardware.
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include "../mesh_node/link_health.h"

static uint32_t lcg = 12345;
static float rnd01() { lcg = lcg * 1664525u + 1013904223u; return (lcg >> 8) / 16777216.0f; }

int main(int argc, char** argv) {
  LinkStats s; bool inited = false;
  LinkState prev = LINK_HEALTHY;
  uint32_t firstDegrade = 0, firstDead = 0;

  auto step = [&](uint32_t t, bool got, int rssi) {
    if (!inited) { if (!got) return; link_init(s, rssi, t); inited = true; prev = s.state; return; }
    if (got) link_on_rx(s, (int8_t)rssi, t);
    link_on_tx(s, got, t);
    link_tick(s, t);
    if (s.state != prev) {
      printf(">>> t=%5.1fs  %s -> %s   (ewma %.1f dBm, slope %.2f dB/s, PRR %.2f, q_pred %.2f)\n",
             t / 1000.0, link_state_name(prev), link_state_name(s.state), s.ewma, s.slope, s.prr, s.qPred);
      if (s.state == LINK_DEGRADING && !firstDegrade) firstDegrade = t;
      if (s.state == LINK_DEAD && !firstDead) firstDead = t;
      prev = s.state;
    }
    if (t % 4000 == 0)
      printf("t=%5.1fs rssi=%4d ewma=%6.1f slope=%6.2f PRR=%.2f dPRR=%5.2f q=%3u%% q_pred=%.2f %s\n",
             t / 1000.0, got ? rssi : -999, s.ewma, s.slope, s.prr, s.dprr, s.health, s.qPred, link_state_name(s.state));
  };

  bool interference = (argc > 1 && strcmp(argv[1], "--interference") == 0);
  if (interference) {
    // Signal strength stays GOOD (-55 dBm) but interference makes packets fail more and more.
    // RSSI slope sees nothing here - only the delivery trend (dPRR) can warn early.
    for (uint32_t t = 0; t <= 60000; t += 500) {
      float x = t / 1000.0f;
      float lossProb = (x < 20) ? 0.0f : (x < 40 ? (x - 20) / 20.0f : 1.0f);
      bool got = rnd01() >= lossProb;
      step(t, got, (int)(-55 + (rnd01() - 0.5f) * 4.0f));
    }
    printf("\nSUMMARY (interference, RSSI constant): Degrading at %.1fs, Dead at %.1fs -> %.1fs warning.\n",
           firstDegrade / 1000.0, firstDead / 1000.0, (firstDead - firstDegrade) / 1000.0);
    return (firstDegrade && firstDegrade < firstDead) ? 0 : 1;
  }
  if (argc > 1) {
    FILE* f = fopen(argv[1], "r"); if (!f) { perror("open"); return 1; }
    char line[128]; long t; int r;
    while (fgets(line, sizeof line, f)) {
      if (sscanf(line, "%ld,%d", &t, &r) == 2) step((uint32_t)t, r > -200, r);
      else if (sscanf(line, "%ld,", &t) == 1) step((uint32_t)t, false, 0);
    }
    fclose(f);
  } else {
    // 0-20s steady -50 | 20-40s falls 2 dB/s to -90 | 40-60s link gone
    for (uint32_t t = 0; t <= 60000; t += 500) {
      float x = t / 1000.0f, truth;
      if (x < 20) truth = -50; else if (x < 40) truth = -50 - 2.0f * (x - 20); else truth = -95;
      float noisy = truth + (rnd01() - 0.5f) * 4.0f;                 // +-2 dB noise
      float lossProb = (truth > -80) ? 0.0f : (truth > -90 ? (-80 - truth) / 10.0f * 0.9f : 1.0f);
      bool got = rnd01() >= lossProb;
      step(t, got, (int)noisy);
    }
    printf("\nSUMMARY: Degrading flagged at %.1fs, Dead at %.1fs -> %.1fs of warning before failure.\n",
           firstDegrade / 1000.0, firstDead / 1000.0, (firstDead - firstDegrade) / 1000.0);
    if (!(firstDegrade && firstDead && firstDegrade < firstDead)) { printf("FAIL\n"); return 1; }
    printf("PASS\n");
  }
  return 0;
}

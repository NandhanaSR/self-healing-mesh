#include "routing.h"
#include <string.h>

struct AdvCell { float cost; uint8_t hops; };

static const uint8_t HOP_INF = 255;

static float      linkCost[MAX_NODES];              // our cost to reach each neighbor directly
static AdvCell    adv[MAX_NODES][MAX_NODES];        // adv[n][d] = what neighbor n says about dest d
static RouteEntry table[MAX_NODES];

// Cost of reaching dest d through neighbor n (ROUTE_INF if impossible)
static float candCost(uint8_t n, uint8_t d) {
  if (n >= MAX_NODES || n == MY_ID || linkCost[n] >= ROUTE_INF) return ROUTE_INF;
  float c;
  if (d == n) c = linkCost[n];
  else if (adv[n][d].cost < ROUTE_INF) c = linkCost[n] + adv[n][d].cost;
  else return ROUTE_INF;
  return c < ROUTE_INF ? c : ROUTE_INF;
}

static bool recompute() {
  RouteEntry old[MAX_NODES];
  memcpy(old, table, sizeof(table));

  for (uint8_t d = 0; d < MAX_NODES; d++) {
    RouteEntry r = { ROUTE_INF, HOP_INF, NO_ROUTE, NO_ROUTE };
    if (d == MY_ID) { r.cost = 0.0f; r.hops = 0; r.nextHop = MY_ID; r.nextHopPrio = MY_ID; table[d] = r; continue; }

    float bestHopsCost = ROUTE_INF;   // cost of the best fewest-hop path (tie-break)
    for (uint8_t n = 0; n < MAX_NODES; n++) {
      if (n == MY_ID || linkCost[n] >= ROUTE_INF) continue;   // dead/unknown link: unusable

      float c = ROUTE_INF;
      uint8_t h = HOP_INF;
      if (d == n) { c = linkCost[n]; h = 1; }
      else {
        if (adv[n][d].cost < ROUTE_INF) c = linkCost[n] + adv[n][d].cost;
        if (adv[n][d].hops < HOP_INF) h = 1 + adv[n][d].hops;
      }
      if (c < ROUTE_INF && c < r.cost) { r.cost = c; r.nextHop = n; }
      if (h < HOP_INF && (h < r.hops || (h == r.hops && c < bestHopsCost))) {
        r.hops = h; r.nextHopPrio = n; bestHopsCost = (c < ROUTE_INF) ? c : ROUTE_INF;
      }
    }
#if USE_ROUTE_HYSTERESIS
    // eq. 15: stay on the current next hop unless the new path is better by more than the margin.
    // (If the current next hop is unusable, candCost() is INF and we switch immediately.)
    uint8_t cur = old[d].nextHop;
    if (cur != NO_ROUTE && cur != r.nextHop && r.nextHop != NO_ROUTE) {
      float cc = candCost(cur, d);
      if (cc < ROUTE_INF && (cc - r.cost) <= ROUTE_HYST_MARGIN) {
        r.nextHop = cur;
        r.cost = cc;
      }
    }
#endif
    table[d] = r;
  }
  return memcmp(old, table, sizeof(table)) != 0;
}

void routing_init() {
  for (uint8_t i = 0; i < MAX_NODES; i++) {
    linkCost[i] = ROUTE_INF;
    for (uint8_t d = 0; d < MAX_NODES; d++) adv[i][d] = { ROUTE_INF, HOP_INF };
  }
  recompute();
}

bool routing_set_link_state(uint8_t nid, LinkState st, float penalty) {
  if (nid >= MAX_NODES || nid == MY_ID) return false;
  float c;
  if (st == LINK_HEALTHY) {
    c = COST_HEALTHY;
  } else if (st == LINK_DEGRADING) {
    c = COST_HEALTHY + penalty;
    if (c >= ROUTE_INF) c = ROUTE_INF - 1.0f;
  } else {
    c = ROUTE_INF;
  }
  if (linkCost[nid] == c) return false;
  linkCost[nid] = c;
  if (st == LINK_DEAD)
    for (uint8_t d = 0; d < MAX_NODES; d++) adv[nid][d] = { ROUTE_INF, HOP_INF };  // forget its routes
  return recompute();
}

bool routing_on_adv(uint8_t from, const RouteAdvEntry* e, uint8_t n) {
  if (from >= MAX_NODES || from == MY_ID) return false;
  for (uint8_t d = 0; d < MAX_NODES; d++) adv[from][d] = { ROUTE_INF, HOP_INF };  // omitted = unreachable
  for (uint8_t i = 0; i < n && i < MAX_NODES; i++) {
    if (e[i].dest >= MAX_NODES || e[i].dest == MY_ID) continue;
    adv[from][e[i].dest] = { e[i].cost, e[i].hops };
  }
  return recompute();
}

uint8_t routing_next_hop(uint8_t dest, bool priority) {
  if (dest >= MAX_NODES) return NO_ROUTE;
  return priority ? table[dest].nextHopPrio : table[dest].nextHop;
}

uint8_t routing_build_adv(uint8_t to, RouteAdvEntry* out) {
  uint8_t k = 0;
  for (uint8_t d = 0; d < MAX_NODES; d++) {
    if (d == MY_ID || d == to) continue;
    float cost = (table[d].nextHop == to) ? ROUTE_INF : table[d].cost;       // poison reverse
    uint8_t hops = (table[d].nextHopPrio == to) ? HOP_INF : table[d].hops;
    if (cost >= ROUTE_INF && hops >= HOP_INF) continue;
    out[k++] = { d, cost, hops };
  }
  return k;
}

const RouteEntry* routing_entry(uint8_t dest) { return dest < MAX_NODES ? &table[dest] : nullptr; }

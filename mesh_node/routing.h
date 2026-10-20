// routing.h - Module 3+4: distance-vector routing with priority override.
// Pure C++ (testable on a laptop).
//
// How it works (RIP-style, RFC 2453):
//  * We store the latest route advertisement from each neighbor, plus the cost of
//    our own link to that neighbor (Healthy=1, Degrading=1+penalty, Dead=unreachable).
//  * Every time anything changes we recompute all routes from scratch
//    (Bellman-Ford over neighbors), so a dead/degrading link is handled immediately.
//  * Two answers per destination:
//      nextHop      = lowest COST path   (normal traffic, avoids Degrading links)
//      nextHopPrio  = fewest HOPS path   (urgent traffic, ignores the penalty)
//  * Poison reverse: we never advertise a route back to the neighbor we learned it from.
//  * Route-switch hysteresis (TinyHeal eq. 15): we only leave the current next hop if the new
//    path is cheaper by MORE than ROUTE_HYST_MARGIN - unless the current one is dead/unreachable,
//    in which case we switch immediately. Stops routes flapping on small fluctuations.
//  * The penalty for a Degrading link comes from predicted quality q_pred (link_penalty()).
#pragma once
#include <stdint.h>
#include "config.h"
#include "link_health.h"
#include "packets.h"

struct RouteEntry {
  float   cost;
  uint8_t hops;
  uint8_t nextHop;       // NO_ROUTE if unreachable
  uint8_t nextHopPrio;
};

void routing_init();
bool routing_set_link_state(uint8_t neighborId, LinkState st, float penalty = COST_PENALTY); // true if table changed
bool routing_on_adv(uint8_t fromId, const RouteAdvEntry* e, uint8_t n); // true if table changed
uint8_t routing_next_hop(uint8_t dest, bool priority);                  // NO_ROUTE if none
uint8_t routing_build_adv(uint8_t toNeighbor, RouteAdvEntry* out);      // returns entry count
const RouteEntry* routing_entry(uint8_t dest);

// neighbor_table.h - Module 1: who is around us, and their link stats.
// Indexed directly by node id (0..MAX_NODES-1), so no searching needed.
#pragma once
#include "config.h"
#include "link_health.h"

struct Neighbor {
  bool      active;          // heard at least once
  uint8_t   mac[6];
  LinkStats link;
  LinkState reportedState;   // last state we told the routing module about
};

void      nt_init();
Neighbor* nt_get(uint8_t id);                                   // nullptr if out of range
bool      nt_add(uint8_t id, const uint8_t* mac, int8_t rssi, uint32_t now); // true if new
void      nt_heard(uint8_t id, int8_t rssi, uint32_t now);      // update link on any rx
int       nt_id_from_mac(const uint8_t* mac);                   // -1 if unknown

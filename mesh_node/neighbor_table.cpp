#include "neighbor_table.h"
#include <string.h>

static Neighbor g_nbr[MAX_NODES];

void nt_init() { memset(g_nbr, 0, sizeof(g_nbr)); }

Neighbor* nt_get(uint8_t id) { return id < MAX_NODES ? &g_nbr[id] : nullptr; }

bool nt_add(uint8_t id, const uint8_t* mac, int8_t rssi, uint32_t now) {
  if (id >= MAX_NODES || id == MY_ID) return false;
  Neighbor& n = g_nbr[id];
  bool isNew = !n.active;
  memcpy(n.mac, mac, 6);
  if (isNew) {
    n.active = true;
    link_init(n.link, rssi, now);
    n.reportedState = LINK_DEAD;   // forces a "link up" event the first time we sync
  } else {
    link_on_rx(n.link, rssi, now);
  }
  return isNew;
}

void nt_heard(uint8_t id, int8_t rssi, uint32_t now) {
  Neighbor* n = nt_get(id);
  if (n && n->active) link_on_rx(n->link, rssi, now);
}

int nt_id_from_mac(const uint8_t* mac) {
  for (uint8_t i = 0; i < MAX_NODES; i++)
    if (g_nbr[i].active && memcmp(g_nbr[i].mac, mac, 6) == 0) return i;
  return -1;
}

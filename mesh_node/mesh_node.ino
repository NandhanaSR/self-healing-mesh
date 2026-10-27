// mesh_node.ino - orchestration only. All logic lives in the modules.
//
// Flash the SAME sketch to every board; change only MY_ID in config.h.
// Serial monitor @115200. Type a digit (0-5) to choose the data destination,
// type 'p' to toggle PRIORITY (urgent) mode.
#include "config.h"
#include "packets.h"
#include "link_health.h"
#include "neighbor_table.h"
#include "routing.h"
#include "anomaly.h"
#include "transport.h"
#include "sensors.h"

static AnomalyDetector anom1, anom2;
static uint16_t g_seq = 0;
static uint8_t  g_dest = DEFAULT_DEST_ID;
static bool     g_prio = false;
static bool     g_advPending = false;
static uint32_t tHello = 0, tHb = 0, tAdv = 0, tData = 0, tPrint = 0;
static uint32_t tLastTrigAdv = 0;

static void fillHeader(PacketHeader& h, uint8_t type, uint8_t dest, uint8_t prio) {
  h.type = type; h.src = MY_ID; h.dest = dest; h.prio = prio; h.seq = g_seq++; h.ttl = MAX_TTL;
}

static bool usable(uint8_t id) {
  Neighbor* n = nt_get(id);
  return n && n->active && n->link.state != LINK_DEAD;
}

// Tell the routing module whenever a link changes state (this is the "event-triggered" update).
static void syncLinkState(uint8_t id) {
  Neighbor* n = nt_get(id);
  if (!n || !n->active) return;
  uint32_t now = millis();
  float penalty = link_penalty(n->link);

  // Path A: state changed -> same as before, push immediately
  if (n->link.state != n->reportedState) {
    Serial.printf("[LINK] node %u: %s -> %s  (ewma %.1f dBm, slope %.2f dB/s, PRR %.2f, q %u%%, q_pred %.2f)\n",
                  id, link_state_name(n->reportedState), link_state_name(n->link.state),
                  n->link.ewma, n->link.slope, n->link.prr, n->link.health, n->link.qPred);
    n->reportedState = n->link.state;
    n->reportedCost = (n->link.state == LINK_DEAD)    ? ROUTE_INF
                    : (n->link.state == LINK_HEALTHY) ? COST_HEALTHY
                                                      : COST_HEALTHY + penalty;
    if (routing_set_link_state(id, n->link.state, penalty)) { g_advPending = true; tLastTrigAdv = now; }
    return;
  }

  // Path B: still DEGRADING -> keep the penalty tracking q_pred
  if (n->link.state == LINK_DEGRADING) {
    bool changed = routing_set_link_state(id, LINK_DEGRADING, penalty);   // always applied locally
    float cost = COST_HEALTHY + penalty;
    if (changed && fabsf(cost - n->reportedCost) >= COST_ADV_DELTA
                && now - tLastTrigAdv >= ADV_MIN_GAP_MS) {
      n->reportedCost = cost;
      g_advPending = true;
      tLastTrigAdv = now;
    }
  }
}

static void sendHello() { PacketHeader h; fillHeader(h, PKT_HELLO, 0xFF, 0); transport_broadcast(&h, sizeof(h)); }

static void sendHeartbeats() {
  for (uint8_t id = 0; id < MAX_NODES; id++) {
    if (!usable(id)) continue;
    PacketHeader h; fillHeader(h, PKT_HEARTBEAT, id, 0);
    transport_send(nt_get(id)->mac, &h, sizeof(h));
  }
}

static void sendAdverts() {
  for (uint8_t id = 0; id < MAX_NODES; id++) {
    if (!usable(id)) continue;
    RouteAdvPacket p; fillHeader(p.h, PKT_ROUTE_ADV, id, 0);
    p.count = routing_build_adv(id, p.e);
    transport_send(nt_get(id)->mac, &p, sizeof(PacketHeader) + 1 + p.count * sizeof(RouteAdvEntry));
  }
  g_advPending = false;
}

static void forwardData(DataPacket& p) {
  uint8_t nh = routing_next_hop(p.h.dest, p.h.prio == 1);
  if (nh == NO_ROUTE || !usable(nh)) { Serial.printf("[FWD] no route to %u\n", p.h.dest); return; }
  Serial.printf("[FWD] dest %u via %u%s\n", p.h.dest, nh, p.h.prio ? " (PRIORITY)" : "");
  transport_send(nt_get(nh)->mac, &p, sizeof(p));
}

static void handleRx(const RxItem& rx) {
  if (rx.len < sizeof(PacketHeader)) return;
  PacketHeader h; memcpy(&h, rx.data, sizeof(h));
  uint32_t now = millis();

  int sender = nt_id_from_mac(rx.mac);
  if (sender < 0) {                                   // first time we hear this board
    if (h.type == PKT_DATA || h.src >= MAX_NODES || h.src == MY_ID) return;
    sender = h.src;
    transport_add_peer(rx.mac);
    nt_add(sender, rx.mac, rx.rssi, now);
  } else {
    nt_heard(sender, rx.rssi, now);
  }
  syncLinkState(sender);

  if (h.type == PKT_ROUTE_ADV && rx.len >= sizeof(PacketHeader) + 1) {
    RouteAdvPacket p; memset(&p, 0, sizeof(p));
    memcpy(&p, rx.data, rx.len < sizeof(p) ? rx.len : sizeof(p));
    if (routing_on_adv(sender, p.e, p.count)) g_advPending = true;
  } else if (h.type == PKT_DATA && rx.len >= sizeof(DataPacket)) {
    DataPacket p; memcpy(&p, rx.data, sizeof(p));
    if (p.h.dest == MY_ID) {
      Serial.printf("[DATA] from %u  s1=%d s2=%d  anomaly=0x%02X/0x%02X%s\n", p.h.src, p.sensor1, p.sensor2,
                    p.anomaly1, p.anomaly2, p.h.prio ? "  PRIORITY" : "");
    } else if (p.h.ttl > 1) { p.h.ttl--; forwardData(p); }
  }
}

static void sendSensorData() {
  if (g_dest == MY_ID) return;
  DataPacket p; fillHeader(p.h, PKT_DATA, g_dest, g_prio ? 1 : 0);
  p.ts = millis();
  p.sensor1 = sensor_read(0); p.sensor2 = sensor_read(1);
  p.anomaly1 = anomaly_update(anom1, p.sensor1);      // tagged on the data, NOT used for routing
  p.anomaly2 = anomaly_update(anom2, p.sensor2);
  forwardData(p);
}

static void printTables() {
  Serial.printf("---- node %u ----\n", MY_ID);
  for (uint8_t id = 0; id < MAX_NODES; id++) {
    Neighbor* n = nt_get(id);
    if (n && n->active)
      Serial.printf("  nbr %u  %-9s ewma %.1f  slope %.2f  PRR %.2f dPRR %.2f  q %u%% q_pred %.2f\n", id,
                    link_state_name(n->link.state), n->link.ewma, n->link.slope, n->link.prr, n->link.dprr,
                    n->link.health, n->link.qPred);
  }
  for (uint8_t d = 0; d < MAX_NODES; d++) {
    const RouteEntry* r = routing_entry(d);
    if (d == MY_ID || r->nextHop == NO_ROUTE) continue;
    Serial.printf("  route -> %u  next %u (cost %.2f) | priority next %u (hops %u)\n", d, r->nextHop, r->cost, r->nextHopPrio, r->hops);
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  if (!transport_init()) { Serial.println("ESP-NOW init FAILED"); while (true) delay(1000); }
  nt_init(); routing_init(); sensors_init(); anomaly_init(anom1); anomaly_init(anom2);
  Serial.printf("Node %u ready. Type 0-5 = destination, p = toggle priority.\n", MY_ID);
}

void loop() {
  uint32_t now = millis();

  RxItem rx;   while (transport_pop_rx(rx)) handleRx(rx);
  TxResult tx; while (transport_pop_tx(tx)) {
    int id = nt_id_from_mac(tx.mac);
    if (id >= 0) { link_on_tx(nt_get(id)->link, tx.ok, now); syncLinkState(id); }
  }
  for (uint8_t id = 0; id < MAX_NODES; id++) {         // catches neighbors that went completely silent
    Neighbor* n = nt_get(id);
    if (n && n->active) { link_tick(n->link, now); syncLinkState(id); }
  }

  if (now - tHello >= HELLO_INTERVAL_MS)     { tHello = now; sendHello(); }
  if (now - tHb    >= HEARTBEAT_INTERVAL_MS) { tHb = now;    sendHeartbeats(); }
  if (g_advPending || now - tAdv >= ADV_INTERVAL_MS) { tAdv = now; sendAdverts(); }
  if (now - tData  >= DATA_INTERVAL_MS)      { tData = now;  sendSensorData(); }
  if (now - tPrint >= PRINT_INTERVAL_MS)     { tPrint = now; printTables(); }

  while (Serial.available()) {
    char c = Serial.read();
    if (c >= '0' && c < '0' + MAX_NODES) { g_dest = c - '0'; Serial.printf("destination = %u\n", g_dest); }
    else if (c == 'p') { g_prio = !g_prio; Serial.printf("priority = %s\n", g_prio ? "ON" : "OFF"); }
  }
}

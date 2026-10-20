// packets.h - the wire format. Every packet starts with the same header,
// so the receiver always looks at byte 0 (type) first instead of guessing by size.
#pragma once
#include <stdint.h>
#include "config.h"

enum PacketType : uint8_t { PKT_HELLO = 0, PKT_HEARTBEAT = 1, PKT_DATA = 2, PKT_ROUTE_ADV = 3 };

#define ANOM_SPIKE 0x01
#define ANOM_STUCK 0x02
#define ANOM_JUMP  0x04

struct __attribute__((packed)) PacketHeader {
  uint8_t  type;
  uint8_t  src;     // original sender (node id)
  uint8_t  dest;    // final destination (node id), 0xFF = broadcast
  uint8_t  prio;    // 0 normal, 1 urgent (uses fewest-hop route)
  uint16_t seq;
  uint8_t  ttl;
};

struct __attribute__((packed)) DataPacket {
  PacketHeader h;
  uint32_t ts;
  int16_t  sensor1, sensor2;
  uint8_t  anomaly1, anomaly2;   // ANOM_* flags - tagged on data, never used for routing
};

struct __attribute__((packed)) RouteAdvEntry {
  uint8_t dest;
  float   cost;   // best cost (hops + degrading penalty)
  uint8_t hops;   // fewest hops (ignores penalty) - used by priority traffic
};

struct __attribute__((packed)) RouteAdvPacket {
  PacketHeader  h;
  uint8_t       count;
  RouteAdvEntry e[MAX_NODES];
};

static_assert(sizeof(DataPacket) <= 250, "ESP-NOW payload limit is 250 bytes");
static_assert(sizeof(RouteAdvPacket) <= 250, "ESP-NOW payload limit is 250 bytes");

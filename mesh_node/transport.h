// transport.h - the ONLY file that talks to ESP-NOW. (Arduino/ESP32 core 3.x)
// The receive callback just drops packets into a queue; loop() processes them.
#pragma once
#include <Arduino.h>

struct RxItem { uint8_t mac[6]; int8_t rssi; uint8_t len; uint8_t data[250]; };
struct TxResult { uint8_t mac[6]; bool ok; };

bool transport_init();
bool transport_add_peer(const uint8_t* mac);
bool transport_send(const uint8_t* mac, const void* data, size_t len);
bool transport_broadcast(const void* data, size_t len);
bool transport_pop_rx(RxItem& out);
bool transport_pop_tx(TxResult& out);
void transport_my_mac(uint8_t* out6);

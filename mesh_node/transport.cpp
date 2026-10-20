#include "transport.h"
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "config.h"

static const uint8_t BCAST[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

#define RXQ 8
#define TXQ 16
static RxItem   rxq[RXQ];  static volatile uint8_t rxHead = 0, rxTail = 0;
static TxResult txq[TXQ];  static volatile uint8_t txHead = 0, txTail = 0;
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// Callbacks run in the WiFi task: keep them tiny (copy + return, no Serial prints).
static void onSent(const wifi_tx_info_t* info, esp_now_send_status_t status) {
  portENTER_CRITICAL(&mux);
  uint8_t next = (txHead + 1) % TXQ;
  if (next != txTail) { memcpy(txq[txHead].mac, info->des_addr, 6); txq[txHead].ok = (status == ESP_NOW_SEND_SUCCESS); txHead = next; }
  portEXIT_CRITICAL(&mux);
}

static void onRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  if (len <= 0 || len > 250) return;
  portENTER_CRITICAL(&mux);
  uint8_t next = (rxHead + 1) % RXQ;
  if (next != rxTail) {
    memcpy(rxq[rxHead].mac, info->src_addr, 6);
    rxq[rxHead].rssi = info->rx_ctrl->rssi;
    rxq[rxHead].len = len;
    memcpy(rxq[rxHead].data, data, len);
    rxHead = next;
  }
  portEXIT_CRITICAL(&mux);
}

bool transport_init() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();                                   // no router, no access point
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() != ESP_OK) return false;
  esp_now_register_send_cb(onSent);
  esp_now_register_recv_cb(onRecv);
  return transport_add_peer(BCAST);
}

bool transport_add_peer(const uint8_t* mac) {
  if (esp_now_is_peer_exist(mac)) return true;
  esp_now_peer_info_t p = {};
  memcpy(p.peer_addr, mac, 6);
  p.channel = WIFI_CHANNEL;
  p.encrypt = false;
  return esp_now_add_peer(&p) == ESP_OK;
}

bool transport_send(const uint8_t* mac, const void* data, size_t len) {
  return esp_now_send(mac, (const uint8_t*)data, len) == ESP_OK;
}
bool transport_broadcast(const void* data, size_t len) { return transport_send(BCAST, data, len); }

bool transport_pop_rx(RxItem& out) {
  bool ok = false;
  portENTER_CRITICAL(&mux);
  if (rxTail != rxHead) { out = rxq[rxTail]; rxTail = (rxTail + 1) % RXQ; ok = true; }
  portEXIT_CRITICAL(&mux);
  return ok;
}
bool transport_pop_tx(TxResult& out) {
  bool ok = false;
  portENTER_CRITICAL(&mux);
  if (txTail != txHead) { out = txq[txTail]; txTail = (txTail + 1) % TXQ; ok = true; }
  portEXIT_CRITICAL(&mux);
  return ok;
}
void transport_my_mac(uint8_t* out6) { WiFi.macAddress(out6); }

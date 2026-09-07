#pragma once
#include <stdint.h>
#include <stddef.h>
using esp_err_t = int;
constexpr esp_err_t ESP_OK = 0;
constexpr esp_err_t ESP_ERR_ESPNOW_EXIST = 1;
constexpr int WIFI_IF_STA = 0;
enum esp_now_send_status_t { ESP_NOW_SEND_SUCCESS, ESP_NOW_SEND_FAIL };
struct wifi_tx_info_t {};
struct RxControl { int8_t rssi = -55; };
struct esp_now_recv_info_t { const uint8_t* src_addr; RxControl* rx_ctrl; };
struct esp_now_peer_info_t { uint8_t peer_addr[6]; uint8_t channel; int ifidx; bool encrypt; };
using SendCallback = void(*)(const wifi_tx_info_t*, esp_now_send_status_t);
using ReceiveCallback = void(*)(const esp_now_recv_info_t*, const uint8_t*, int);
inline SendCallback testSendCallback = nullptr;
inline ReceiveCallback testReceiveCallback = nullptr;
inline esp_err_t esp_now_init() { return ESP_OK; }
inline esp_err_t esp_now_deinit() { return ESP_OK; }
inline esp_err_t esp_now_register_send_cb(SendCallback callback) {
  testSendCallback = callback; return ESP_OK;
}
inline esp_err_t esp_now_register_recv_cb(ReceiveCallback callback) {
  testReceiveCallback = callback; return ESP_OK;
}
inline void esp_now_unregister_send_cb() { testSendCallback = nullptr; }
inline void esp_now_unregister_recv_cb() { testReceiveCallback = nullptr; }
inline bool esp_now_is_peer_exist(const uint8_t*) { return true; }
inline esp_err_t esp_now_add_peer(const esp_now_peer_info_t*) { return ESP_OK; }
esp_err_t esp_now_send(const uint8_t*, const uint8_t*, size_t);

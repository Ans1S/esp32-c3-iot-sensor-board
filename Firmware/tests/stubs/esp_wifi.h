#pragma once
#include "esp_now.h"
constexpr int WIFI_SECOND_CHAN_NONE = 0;
esp_err_t esp_wifi_set_channel(uint8_t channel, int);
inline esp_err_t esp_wifi_set_max_tx_power(int8_t) { return ESP_OK; }

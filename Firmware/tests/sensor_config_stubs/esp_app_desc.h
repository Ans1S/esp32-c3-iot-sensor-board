#pragma once
#include <stdint.h>
struct esp_app_desc_t { uint8_t app_elf_sha256[32]{}; };
inline esp_app_desc_t sensorConfigDescription;
inline const esp_app_desc_t* esp_app_get_description() { return &sensorConfigDescription; }

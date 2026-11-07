#pragma once
#include <cstdint>
#include <cstring>
// Ordering/integrity substitute, matching the client control-flow fixture.
// Real SHA-256/ECDSA package tests and target mbedTLS builds remain separate.
struct mbedtls_sha256_context { uint32_t value = 2166136261U; };
inline void mbedtls_sha256_init(mbedtls_sha256_context* h) { h->value = 2166136261U; }
inline void mbedtls_sha256_free(mbedtls_sha256_context*) {}
inline int mbedtls_sha256_starts(mbedtls_sha256_context* h, int) { mbedtls_sha256_init(h); return 0; }
inline int mbedtls_sha256_update(mbedtls_sha256_context* h, const uint8_t* bytes, size_t size) {
  while (size--) h->value = (h->value ^ *bytes++) * 16777619U; return 0;
}
inline int mbedtls_sha256_finish(mbedtls_sha256_context* h, uint8_t* output) {
  for (int i = 0; i < 8; ++i) memcpy(output + i * 4, &h->value, 4); return 0;
}

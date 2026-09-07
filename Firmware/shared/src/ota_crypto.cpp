#include "ota_protocol.h"
#include <mbedtls/pk.h>
#include <mbedtls/sha256.h>
#if __has_include("ota_public_key.h")
#include "ota_public_key.h"
#else
// Fail closed until an installation-specific public key has been provisioned.
static constexpr char kOtaPublicKey[] = "";
#endif

bool lil::ota::verifyManifest(const Manifest& m) {
  if (!validManifest(m) || !kOtaPublicKey[0]) return false;
  uint8_t digest[32];
  if (mbedtls_sha256(reinterpret_cast<const uint8_t*>(&m),
                     offsetof(Manifest, signatureSize), digest, 0) != 0) return false;
  mbedtls_pk_context key;
  mbedtls_pk_init(&key);
  const bool ok = mbedtls_pk_parse_public_key(&key,
      reinterpret_cast<const uint8_t*>(kOtaPublicKey), sizeof(kOtaPublicKey)) == 0 &&
      mbedtls_pk_verify(&key, MBEDTLS_MD_SHA256, digest, sizeof(digest),
                         m.signature, m.signatureSize) == 0;
  mbedtls_pk_free(&key);
  return ok;
}

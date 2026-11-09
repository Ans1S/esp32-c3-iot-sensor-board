#ifdef NDEBUG
#error "Regression assertions must be enabled"
#endif

#include <cassert>
#include <string>
#include "http_response_body.h"

void testRadioTick() { ++testMillis; }

namespace {
struct Wire {
  std::string bytes;
  uint32_t periodMs = 0;
  bool keepOpen = false;
  size_t fragment = 512, offset = 0, reads = 0;
  uint32_t start = millis();

  int available() const {
    const size_t arrived = periodMs ? std::min(bytes.size(), size_t((millis() - start) / periodMs)) : bytes.size();
    return static_cast<int>(std::min(arrived - offset, fragment));
  }
  int read(uint8_t* destination, size_t count) {
    assert(count <= size_t(available()));
    memcpy(destination, bytes.data() + offset, count);
    offset += count;
    ++reads;
    return static_cast<int>(count);
  }
  bool connected() const { return offset < bytes.size() || keepOpen; }
};

struct Text {
  std::string bytes;
  size_t largestReserve = 0, reserves = 0;
  bool noMemory = false;

  void operator=(const char* source) { bytes = source; }
  size_t length() const { return bytes.size(); }
  bool reserve(size_t count) {
    ++reserves;
    largestReserve = std::max(largestReserve, count);
    if (noMemory) return false;
    bytes.reserve(count);
    return true;
  }
  bool concat(const char* source, size_t count) {
    bytes.append(source, count);
    return true;
  }
};
}  // namespace

int main() {
  using station::HttpBodyStatus;
  using station::HttpResponseBody;
  Text output;
  const std::string data = "[\"synthetic\",123]";

  Wire fixed{data, 0, true, 3};
  assert(HttpResponseBody(fixed, output, 64, 100).read(data.size(), false) == HttpBodyStatus::Ok);
  assert(output.bytes == data && output.reserves == 1 && output.largestReserve == data.size());
  Wire closeDelimited{data, 1, false, 3};
  assert(HttpResponseBody(closeDelimited, output, 64, 100).read(-1, false) == HttpBodyStatus::Ok);
  assert(output.bytes == data);

  Wire chunks{"3;test=yes\r\n[\"s\r\n7\r\nyntheti\r\n7\r\nc\",123]\r\n0\r\nX-Test: fixture\r\n\r\n", 1, true, 3};
  assert(HttpResponseBody(chunks, output, data.size(), 100).read(-1, true) == HttpBodyStatus::Ok);
  assert(output.bytes == data);
  Wire uppercase{"A\r\n0123456789\r\n0\r\n\r\n"};
  assert(HttpResponseBody(uppercase, output, 10, 100).read(-1, true) == HttpBodyStatus::Ok);
  assert(output.bytes == "0123456789");

  Wire oversized{std::string(1024, 'x')};
  assert(HttpResponseBody(oversized, output, 64, 100).read(1024, false) == HttpBodyStatus::TooLarge);
  assert(oversized.reads == 0 && output.bytes.empty());
  Wire declaredChunk{"10000000000000000000000000000\r\n" + oversized.bytes};
  assert(HttpResponseBody(declaredChunk, output, 64, 100).read(-1, true) == HttpBodyStatus::TooLarge);
  assert(declaredChunk.reads == 1 && declaredChunk.offset == 512 && output.bytes.empty());
  Wire exceededAcrossChunks{"4\r\n1234\r\n4\r\n5678\r\n0\r\n\r\n"};
  assert(HttpResponseBody(exceededAcrossChunks, output, 6, 100).read(-1, true) == HttpBodyStatus::TooLarge);
  assert(output.bytes.empty());
  assert(HttpResponseBody(oversized, output, 64, 100).read(-1, false) == HttpBodyStatus::TooLarge);
  assert(output.bytes.empty());

  Wire largeButAllowed{std::string(32769, 'd'), 0, true, 113};
  Text growing;
  Wire unknownLarge{largeButAllowed.bytes, 0, false, 113};
  assert(HttpResponseBody(unknownLarge, growing, 65536, 100).read(-1, false) == HttpBodyStatus::Ok);
  assert(growing.bytes == largeButAllowed.bytes && growing.reserves == 8 && growing.largestReserve == 65536);
  printf("Unknown-length body: %zu reads, %zu capacity increases for %zu bytes\n",
         unknownLarge.reads, growing.reserves, growing.bytes.size());

  Wire truncated{data};
  assert(HttpResponseBody(truncated, output, 64, 100).read(data.size() + 1, false) == HttpBodyStatus::Truncated);
  Wire missingTrailerTerminator{"0\r\nX-Test: missing\r\n"};
  assert(HttpResponseBody(missingTrailerTerminator, output, 64, 100).read(-1, true) == HttpBodyStatus::Truncated);
  for (const auto& bad : {std::string("nope\r\n"), std::string("2\r\nXYx\r\n"), std::string(150, '0') + "\r\n"}) {
    Wire malformed{bad};
    assert(HttpResponseBody(malformed, output, 64, 100).read(-1, true) == HttpBodyStatus::Invalid);
    assert(output.bytes.empty());
  }

  testMillis = UINT32_MAX - 4;
  Wire trickle{data, 2, true, 1};
  const uint32_t start = millis();
  assert(HttpResponseBody(trickle, output, 64, 7).read(-1, false) == HttpBodyStatus::Timeout);
  assert(millis() - start == 7 && output.bytes.empty() && trickle.reads == 3);
  Wire chunkIdle{"3\r\n123\r\n", 0, true};
  assert(HttpResponseBody(chunkIdle, output, 64, 7).read(-1, true) == HttpBodyStatus::Timeout);
  assert(output.bytes.empty());
  Wire empty{"", 0, true};
  assert(HttpResponseBody(empty, output, 64, 7).read(0, false) == HttpBodyStatus::Ok);
  assert(empty.reads == 0 && output.bytes.empty());
  output.noMemory = true;
  Wire allocationFailure{data};
  assert(HttpResponseBody(allocationFailure, output, 64, 7).read(data.size(), false) == HttpBodyStatus::NoMemory);
  assert(allocationFailure.reads == 0 && output.bytes.empty());
  puts("ThingSpeak body framing: fixed, close-delimited, chunked, limits, errors and absolute deadline passed");
}

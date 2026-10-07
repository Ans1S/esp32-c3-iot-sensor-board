#include <Arduino.h>
#include <Wire.h>
#include <cassert>
#include "precision_sensors.h"
void testRadioTick() { ++testMillis; }
int main() {
  using Type = lil::protocol::EnvironmentalSensorType;
  uint8_t address = 0;
  assert(sensor::PrecisionSensors::probe(Type::kTmp117,address) && address == 0x48);
  Wire.words[15] = 0x1116;
  assert(!sensor::PrecisionSensors::probe(Type::kTmp117,address));
  Wire.words[15] = 0x1117; // Revision bits do not change the part ID.
  assert(sensor::PrecisionSensors::probe(Type::kTmp117,address));
  sensor::PrecisionSensors driver;
  assert(driver.begin(Type::kTmp117,0x48));
  assert(Wire.words[1] == 0x220);
  assert(!driver.read().valid);
  testMillis += 10;
  Wire.words[0] = uint16_t(int16_t(-1280)); Wire.words[1] |= 0x2000;
  auto r = driver.read(); assert(r.valid && r.temperatureC == -10);
  assert(!driver.freshTemperature());
  testMillis += 10;
  Wire.words[0] = 4737; Wire.words[1] |= 0x2000;
  r=driver.read(); assert(r.valid && r.temperatureC == 37.0078125F);
  testMillis += 1501; assert(!driver.read().valid);
  Wire.shortRead = true; assert(!driver.read().valid); Wire.shortRead = false;
  for (uint8_t a = 0x48; a <= 0x4B; ++a) {
    Wire = PrecisionWire{}; Wire.present = a;
    Wire.words[5] = 0x1234; Wire.words[7] = 17; // Factory ID/offset are not rewritten.
    assert(sensor::PrecisionSensors::probe(Type::kTmp117,address) && address == a);
    assert(driver.begin(Type::kTmp117,a));
    assert(Wire.words[5] == 0x1234 && Wire.words[7] == 17);
    for (int16_t raw : {int16_t(-7040),int16_t(19200),int16_t(-32768)}) {
      testMillis += 10; Wire.words[0] = uint16_t(raw); Wire.words[1] |= 0x2000;
      r = driver.read(); assert(r.valid == (raw != -32768));
      if (r.valid) assert(r.temperatureC == raw / 128.0F);
    }
  }
  Wire = PrecisionWire{}; Wire.present = 0x57;
  assert(sensor::PrecisionSensors::probe(Type::kMax30102,address) && address == 0x57);
  Wire.stuckReset = true;
  assert(!driver.begin(Type::kMax30102,address));
  Wire.stuckReset = false; assert(driver.begin(Type::kMax30102,address));
  assert(Wire.registers[10] == 0x27 && Wire.registers[8] == 0x40 && Wire.registers[2] == 0xA0);
  for (int i=0;i<200;++i) {
    testMillis+=40; Wire.sample(100000 + int(1000*sin(i*2*3.141592653589793/12.5)));
    assert(driver.poll());
    if (i%5==0 && i<195) driver.read();
    if (i==100) assert(!(driver.read().capabilities & lil::protocol::kHeartRate));
  }
  r=driver.read();
  assert(r.valid && (r.capabilities & lil::protocol::kHeartRate));
  assert(fabs(r.pulse.beatsPerMinute-120)<2 && r.pulse.status==2);
  assert(r.live.flags & lil::protocol::kLiveEstimateFresh);
  assert(r.live.count > 0 && r.live.count <= 8);
  const auto repeated=driver.read();
  assert(!(repeated.live.flags & lil::protocol::kLiveEstimateFresh));
  assert(repeated.live.count == 0);
  Wire.registers[5]=1; r=driver.read();
  assert(!r.valid && r.pulse.status==3 && !(r.capabilities & lil::protocol::kOptical) &&
      !(r.capabilities & lil::protocol::kHeartRate));
  testMillis+=10; Wire.sample(500); r=driver.read();
  assert(r.pulse.status==0 && !(r.capabilities & lil::protocol::kHeartRate));
  testMillis+=1501; assert(!driver.read().valid);
  Wire.fail=true; assert(!driver.read().valid);
  // Test rates across the supported range, not just one convenient frequency.
  for (int bpm : {35, 45, 60, 72, 100, 120, 150, 180, 195}) {
    Wire = PrecisionWire{}; Wire.present = 0x57;
    assert(driver.begin(Type::kMax30102, 0x57));
    for (int i = 0; i < 225; ++i) {
      testMillis += 40;
      Wire.sample(100000 + int(4000 * sin(i * 2 * 3.141592653589793 * bpm / 1500)));
      assert(driver.poll());
      if (i % 5 == 4) r = driver.read();
    }
    assert((r.capabilities & lil::protocol::kHeartRate) && fabs(r.pulse.beatsPerMinute - bpm) < 2);
    // ALC overflow can occur with plausible ADC counts; never retain BPM.
    Wire.registers[0] = 0x20; r = driver.read();
    assert(!r.valid && !r.live.count && !(r.capabilities & lil::protocol::kOptical) &&
        !(r.capabilities & lil::protocol::kHeartRate) && (r.live.flags & lil::protocol::kLiveGap));
    // Read again immediately: clearing the fault indication must not revive
    // the ADC value from before the discarded acquisition window.
    assert(!driver.read().valid);
    testMillis += 40; Wire.sample(100000);
    assert(driver.read().valid);
  }
  Wire = PrecisionWire{}; Wire.present = 0x57;
  assert(driver.begin(Type::kMax30102, 0x57));
  for (int i = 0; i < 32; ++i) Wire.sample(100000);
  // At exactly full the 5-bit pointers match and OVF_COUNTER is still zero.
  r = driver.read();
  assert(Wire.fifo.empty() && (r.live.flags & lil::protocol::kLiveGap));
  assert(!(r.capabilities & lil::protocol::kHeartRate));
  Wire.registers[0] = 1; // Brownout: require complete configuration again.
  assert(!driver.read().valid);
  Wire.sample(100000); assert(!driver.read().valid);
  assert(driver.begin(Type::kMax30102, 0x57));
  testMillis += 1300; Wire.sample(100000);
  r = driver.read();
  assert(!r.valid && !r.live.count && (r.live.flags & lil::protocol::kLiveGap));
  for (uint8_t received = 1; received < 6; ++received) {
    Wire = PrecisionWire{}; Wire.present = 0x57;
    assert(driver.begin(Type::kMax30102, 0x57));
    Wire.sample(100000); assert(driver.read().valid);
    // The device advances its pointer even if only part of a frame arrives.
    Wire.partialFifoBytes = received; Wire.sample(100000);
    r = driver.read();
    assert(!r.valid && !r.live.count && (r.live.flags & lil::protocol::kLiveGap));
    Wire.partialFifoBytes = 0; Wire.sample(100000);
    assert(!driver.poll() && !driver.read().valid);
    assert(driver.begin(Type::kMax30102, 0x57));
    assert(Wire.fifo.empty());
    Wire.sample(100000); r = driver.read();
    assert(r.valid && !(r.capabilities & lil::protocol::kHeartRate));
  }
  // Heart rate uses IR: a dim red channel must not veto a clean IR pulse.
  Wire = PrecisionWire{}; Wire.present = 0x57;
  assert(driver.begin(Type::kMax30102, 0x57));
  for (int i = 0; i < 250; ++i) {
    testMillis += 40;
    Wire.sample(1500, 100000 + int(1000 * sin(i * 2 * 3.141592653589793 / 25)));
    assert(driver.poll());
    if (i % 5 == 4) r = driver.read();
  }
  assert((r.capabilities & lil::protocol::kHeartRate) && fabs(r.pulse.beatsPerMinute - 60) < 2);
  // Different optical paths require different currents. Coupled LED control
  // oscillated between a bright red channel and a weak IR channel forever.
  Wire = PrecisionWire{}; Wire.present = 0x57;
  assert(driver.begin(Type::kMax30102, 0x57));
  for (int i = 0; i < 600; ++i) {
    testMillis += 40;
    const double modulation = 1 + .01 * sin(i * 2 * 3.141592653589793 / 25);
    Wire.sample(uint32_t(240000 * Wire.registers[0x0C] / 36.0 * modulation),
        uint32_t(30000 * Wire.registers[0x0D] / 36.0 * modulation));
    assert(driver.poll());
    if (i % 5 == 4) r = driver.read();
  }
  assert(Wire.registers[0x0C] < 36 && Wire.registers[0x0D] > 36);
  assert((r.capabilities & lil::protocol::kHeartRate) && fabs(r.pulse.beatsPerMinute - 60) < 2);
  // A real PPG fundamental can be small and slow. Short detrending suppressed
  // this clean 0.2% AC/DC signal below the fixed noise floor.
  Wire = PrecisionWire{}; Wire.present = 0x57;
  assert(driver.begin(Type::kMax30102, 0x57));
  for (int i = 0; i < 250; ++i) {
    testMillis += 40;
    Wire.sample(100000 + int(200 * sin(i * 2 * 3.141592653589793 * 35 / 1500)));
    assert(driver.poll());
    if (i % 5 == 4) r = driver.read();
  }
  assert((r.capabilities & lil::protocol::kHeartRate) && fabs(r.pulse.beatsPerMinute - 35) < 2);
  // Gain recovery must also work below the estimator's contact threshold.
  Wire = PrecisionWire{}; Wire.present = 0x57;
  assert(driver.begin(Type::kMax30102, 0x57));
  for (int i = 0; i < 600; ++i) {
    testMillis += 40;
    Wire.sample(0, uint32_t(6000 * Wire.registers[0x0D] / 36.0 *
        (1 + .02 * sin(i * 2 * 3.141592653589793 / 25))));
    assert(driver.poll());
    if (i % 5 == 4) r = driver.read();
  }
  assert(Wire.registers[0x0C] == 0x24 && Wire.registers[0x0D] == 0x60);
  assert((r.capabilities & lil::protocol::kHeartRate) && fabs(r.pulse.beatsPerMinute - 60) < 2);
  // Static reflection, slow DC drift and random noise are not pulse evidence.
  for (int kind = 0; kind < 3; ++kind) {
    Wire = PrecisionWire{}; Wire.present = 0x57;
    assert(driver.begin(Type::kMax30102, 0x57));
    uint32_t noise = 12345;
    for (int i = 0; i < 300; ++i) {
      noise = 1664525 * noise + 1013904223;
      const int value = 100000 + (kind == 1 ? i * 100 :
          kind == 2 ? int(noise >> 16) % 6001 - 3000 : 0);
      testMillis += 40; Wire.sample(value);
      assert(driver.poll());
      if (i % 5 == 4) {
        r = driver.read();
        assert(!(r.capabilities & lil::protocol::kHeartRate));
      }
    }
  }
  // Any gain change invalidates old waveform/ADC/BPM until new samples arrive.
  Wire = PrecisionWire{}; Wire.present = 0x57;
  assert(driver.begin(Type::kMax30102, 0x57));
  for (int i = 0; i < 25; ++i) {
    testMillis += 40; Wire.sample(100000, 30000);
    assert(driver.poll());
    if (i < 24) driver.read();
  }
  r = driver.read();
  assert(!r.valid && !r.live.count && (r.live.flags & lil::protocol::kLiveGap));
  assert(!driver.read().valid);
  testMillis += 40; Wire.sample(100000, 60000); r = driver.read();
  assert(r.valid && !(r.capabilities & lil::protocol::kHeartRate));
  assert(lil::protocol::liveReportIntervalMs(Type::kLsm6dsox)==100);
  assert(lil::protocol::liveReportIntervalMs(Type::kTmp117)==1000);
  puts("Precision drivers: temperature, pulse rates, separate LED gain, weak IR, dim red, small AC signal, noise rejection, gaps and I2C recovery passed");
}

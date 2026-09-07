#ifdef NDEBUG
#error "Regression assertions must be enabled"
#endif
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "bme68x.h"

struct Bus { uint32_t delayUs = 0; unsigned reads = 0; bool stuck = false; };
static BME68X_INTF_RET_TYPE readRegister(uint8_t reg, uint8_t* data,
                                        uint32_t length, void* context) {
  auto& bus = *static_cast<Bus*>(context);
  ++bus.reads;
  memset(data, 0, length);
  if (reg == BME68X_REG_CTRL_MEAS && bus.stuck) data[0] = BME68X_FORCED_MODE;
  return BME68X_INTF_RET_SUCCESS;
}
static BME68X_INTF_RET_TYPE writeRegister(uint8_t, const uint8_t*, uint32_t, void*) {
  return BME68X_INTF_RET_SUCCESS;
}
static void waitUs(uint32_t period, void* context) {
  static_cast<Bus*>(context)->delayUs += period;
}
int main() {
  Bus bus;
  bme68x_dev device{};
  device.intf = BME68X_I2C_INTF;
  device.intf_ptr = &bus;
  device.read = readRegister;
  device.write = writeRegister;
  device.delay_us = waitUs;
  assert(bme68x_set_op_mode(BME68X_FORCED_MODE, &device) == BME68X_OK);
  assert(bus.reads == 1 && bus.delayUs == 0);
  bus = Bus{}; bus.stuck = true;
  assert(bme68x_set_op_mode(BME68X_FORCED_MODE, &device) == BME68X_E_COM_FAIL);
  assert(bus.reads == 50 && bus.delayUs == 50 * BME68X_PERIOD_POLL);
  bus = Bus{};
  bme68x_data data{};
  uint8_t fields = 99;
  assert(bme68x_get_data(BME68X_FORCED_MODE, &data, &fields, &device) ==
         BME68X_W_NO_NEW_DATA);
  assert(fields == 0 && bus.delayUs <= 50000);
  puts("Patched Bosch driver stuck-mode and missing-data tests passed");
}

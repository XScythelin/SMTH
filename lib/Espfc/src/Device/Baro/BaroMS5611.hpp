#pragma once

#include "Debug_Espfc.h"
#include "Device/BaroDevice.hpp"

namespace Espfc::Device::Baro {

class BaroMS5611 : public BaroDevice
{
public:
  int begin(BusDevice* bus) final;
  int begin(BusDevice* bus, uint8_t addr) final;

  BaroDeviceType getType() const final;

  float readTemperature() final;
  float readPressure() final;

  void setMode(BaroDeviceMode mode) final;
  int getDelay(BaroDeviceMode mode) const final;

  bool testConnection() final;

protected:
  int8_t   sendCommand(uint8_t cmd);
  uint32_t readADC();
  bool     readPROM();
  uint8_t  crc4(uint16_t prom[8]);

  uint16_t _c[6];    // C1..C6 calibration coefficients from PROM (_c[0]=C1 .. _c[5]=C6)
  int32_t  _dT;      // difference between actual and reference temperature
  int32_t  _t_fine;  // compensated temperature * 100, stored by readTemperature()
};

} // namespace Espfc::Device::Baro
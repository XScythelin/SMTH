#pragma once

#include "Debug_Espfc.h"
#include "Device/RangefinderDevice.hpp"

namespace Espfc::Device::Rangefinder {

// Native ST VL53L0X driver (no external library). The initialization
// sequence is ported from the public ST API / pololu reference driver.
class RangefinderVL53L0X : public RangefinderDevice
{
public:
  int begin(BusDevice* bus) final;
  int begin(BusDevice* bus, uint8_t addr) final;

  RangefinderDeviceType getType() const final;

  int32_t readRangeMm() final;
  int getDelay() const final;
  int32_t getMaxRangeMm() const final;

  bool testConnection() final;

protected:
  uint8_t readReg(uint8_t reg);
  uint16_t readReg16(uint8_t reg);
  void readMulti(uint8_t reg, uint8_t* dst, uint8_t count);
  void writeReg(uint8_t reg, uint8_t val);
  void writeReg16(uint8_t reg, uint16_t val);
  void writeMulti(uint8_t reg, const uint8_t* src, uint8_t count);

  bool getSpadInfo(uint8_t* count, bool* typeIsAperture);
  bool performSingleRefCalibration(uint8_t vhvInitByte);

  uint8_t _stopVariable = 0;
};

} // namespace Espfc::Device::Rangefinder

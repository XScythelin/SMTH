#pragma once

#include "BusAwareDevice.hpp"
#include "BusDevice.hpp"

namespace Espfc {

enum RangefinderDeviceType
{
  RANGEFINDER_DEFAULT = 0,
  RANGEFINDER_NONE = 1,
  RANGEFINDER_VL53L0X = 2,
  RANGEFINDER_MSP = 3,
  RANGEFINDER_MAX
};

namespace Device {

class RangefinderDevice : public BusAwareDevice
{
public:
  typedef RangefinderDeviceType DeviceType;

  virtual int begin(BusDevice* bus) = 0;
  virtual int begin(BusDevice* bus, uint8_t addr) = 0;

  virtual DeviceType getType() const = 0;

  // returns measured distance in millimeters, or -1 when no new valid sample is available yet
  virtual int32_t readRangeMm() = 0;

  // polling interval in microseconds
  virtual int getDelay() const = 0;

  // maximum reliable measuring distance in millimeters
  virtual int32_t getMaxRangeMm() const = 0;

  virtual bool testConnection() = 0;

  static const char** getNames();
  static const char* getName(DeviceType type);
};

} // namespace Device

} // namespace Espfc

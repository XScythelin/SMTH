#include "RangefinderDevice.hpp"
#include "Hal/Pgm.h"
#include <cstddef>

namespace Espfc::Device {

const char** RangefinderDevice::getNames()
{
  static const char* devChoices[] = {PSTR("AUTO"), PSTR("NONE"), PSTR("VL53L0X"), PSTR("MSP"), NULL};
  return devChoices;
}

const char* RangefinderDevice::getName(DeviceType type)
{
  if (type >= RANGEFINDER_MAX) return PSTR("?");
  return getNames()[type];
}

} // namespace Espfc::Device

#pragma once

#include "BaseSensor.h"
#include "Device/RangefinderDevice.hpp"
#include "Model.h"
#include "Utils/Filter.h"

namespace Espfc::Sensor {

class RangefinderSensor : public BaseSensor
{
public:
  RangefinderSensor(Model& model);

  int begin();
  int update();
  int read();

private:
  int applySample(int32_t raw);

  Model& _model;
  Device::RangefinderDevice* _rangefinder;
  Utils::Filter _distanceFilter;
  uint32_t _wait;
  int32_t _maxRangeMm;
  bool _isMsp;
  uint32_t _lastExtTs;
};

} // namespace Espfc::Sensor

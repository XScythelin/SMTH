#pragma once

#include "BaseSensor.h"
#include "Device/BaroDevice.hpp"
#include "Model.h"
#include "Utils/Filter.h"

namespace Espfc::Sensor {

class BaroSensor : public BaseSensor
{
public:
  enum BaroState
  {
    BARO_STATE_INIT,
    BARO_STATE_TEMP_GET,
    BARO_STATE_PRESS_GET,
  };

  BaroSensor(Model& model);

  int begin();
  int update();
  int read();

private:
  void readTemperature();
  void readPressure();
  void updateAltitude();
  void updateGroundReference(float pressure, float vario, bool armed);

  Model& _model;
  Device::BaroDevice* _baro;
  BaroState _state;
  Utils::Filter _temperatureFilter;
  Utils::Filter _temperatureMedianFilter;
  Utils::Filter _pressureFilter;
  Utils::Filter _pressureMedianFilter;
  Utils::Filter _altitudeFilter;
  Utils::Filter _varioFilter;
  uint32_t _wait;
  int32_t _counter;
  bool _pressureValid;
  float _groundPressure;
  float _calPressureMin;
  float _calPressureMax;
  float _calPressureSum;
  int32_t _calPressureCount;
  bool _groundCalibrated;
};

} // namespace Espfc::Sensor

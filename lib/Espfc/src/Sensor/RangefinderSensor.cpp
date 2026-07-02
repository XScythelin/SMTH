#include "RangefinderSensor.hpp"
#include <algorithm>

namespace Espfc::Sensor {

RangefinderSensor::RangefinderSensor(Model& model):
  _model(model), _rangefinder(nullptr), _wait(0), _maxRangeMm(0), _isMsp(false), _lastExtTs(0) {}

int RangefinderSensor::begin()
{
  const bool msp = _model.config.rangefinder.dev == RANGEFINDER_MSP && _model.isFeatureActive(FEATURE_RANGEFINDER);
  if (!msp && (!_model.rangefinderActive() || !_model.state.rangefinder.dev)) return 0;

  _isMsp = msp;
  _rangefinder = _model.state.rangefinder.dev; // null for MSP source
  _lastExtTs = 0;

  int rate;
  if (_isMsp)
  {
    rate = 50; // nominal MTF-02 lidar update rate [Hz]
    _maxRangeMm = (int32_t)_model.config.rangefinder.maxRange * 10;
  }
  else
  {
    const int interval = _rangefinder->getDelay();
    rate = 1000000 / interval;
    _maxRangeMm = std::min((int32_t)_model.config.rangefinder.maxRange * 10, _rangefinder->getMaxRangeMm());
  }

  _distanceFilter.begin(_model.config.rangefinder.filter, rate);

  _model.state.rangefinder.rate = rate;
  _model.state.rangefinder.valid = false;
  _model.state.rangefinder.distance = 0.0f;
  _model.state.rangefinder.height = 0.0f;

  const auto type = _isMsp ? RANGEFINDER_MSP : _rangefinder->getType();
  _model.logger.info()
      .log(F("RANGEFINDER INIT"))
      .log(FPSTR(Device::RangefinderDevice::getName(type)))
      .log(rate)
      .logln(_maxRangeMm);

  return 1;
}

int RangefinderSensor::update()
{
  return read();
}

int RangefinderSensor::read()
{
  if (_isMsp)
  {
    if (!_model.isFeatureActive(FEATURE_RANGEFINDER)) return 0;

    Espfc::RangefinderState& rf = _model.state.rangefinder;

    // drop to not-present when the MSP feed goes stale
    constexpr uint32_t MSP_TIMEOUT_US = 200000; // 200 ms
    if (rf.extTs == 0 || (uint32_t)(micros() - rf.extTs) > MSP_TIMEOUT_US)
    {
      rf.present = false;
      rf.valid = false;
      rf.height = 0.0f;
    }

    if (rf.extTs == _lastExtTs) return 0; // no new sample
    _lastExtTs = rf.extTs;
    rf.present = true;

    return applySample(rf.extRaw);
  }

  if (!_rangefinder || !_model.rangefinderActive()) return 0;

  if (_wait > micros()) return 0;
  _wait = micros() + _rangefinder->getDelay();

  const int32_t raw = _rangefinder->readRangeMm();
  if (raw < 0)
  {
    // no new sample ready yet
    return 0;
  }

  return applySample(raw);
}

int RangefinderSensor::applySample(int32_t raw)
{
  Espfc::RangefinderState& rf = _model.state.rangefinder;

  rf.raw = raw;
  rf.valid = raw > 0 && raw <= _maxRangeMm;

  const float distance = _distanceFilter.update((raw > 0 ? raw : 0) * 0.001f); // mm -> m
  rf.distance = distance;

  // tilt-compensate slant distance to vertical height (cosTheta = 1 when level).
  // Reject readings tilted beyond the max angle (Betaflight uses 25 deg -> cos ~0.9063),
  // otherwise the projection produces a meaningless near-zero height.
  constexpr float RANGEFINDER_MAX_TILT_COS = 0.9063f;
  const float cosTheta = _model.state.attitude.cosTheta;
  if (rf.valid && cosTheta >= RANGEFINDER_MAX_TILT_COS)
  {
    rf.height = distance * cosTheta;
  }
  else
  {
    rf.valid = false;
    rf.height = 0.0f;
  }
  rf.updateCount++;

  if (_model.config.debug.mode == DEBUG_RANGEFINDER)
  {
    _model.state.debug[0] = std::clamp(raw, (int32_t)-32000, (int32_t)32000);             // raw mm
    _model.state.debug[1] = std::clamp(lrintf(rf.height * 1000.0f), -32000l, 32000l);     // height mm
    _model.state.debug[2] = rf.valid ? 1 : 0;
  }

  return 1;
}

} // namespace Espfc::Sensor

#pragma once

#include "Model.h"
#include "Utils/Filter.h"
#include <cmath>

namespace Espfc::Control {

class Altitude
{
public:
  Altitude(Model& model): _model(model) {}

  int begin()
  {
    _model.state.altitude.baroHeight = _model.state.baro.altitudeGround;
    _model.state.altitude.accelHeight = _model.state.baro.altitudeGround;
    _model.state.altitude.height = _model.state.baro.altitudeGround;
    _model.state.altitude.vario = _model.state.baro.vario;
    _model.state.altitude.target = _model.state.altitude.height;
    _model.state.altitude.accelVario = 0.0f;
    _model.state.altitude.accelBias = 0.0f;
    _model.state.altitude.baroUpdateCount = _model.state.baro.updateCount;

    _accelFilter.begin(FilterConfig(FILTER_PT1, 20), _model.state.accel.timer.rate);

    return 1;
  }

  int update()
  {
    const float dt = _model.state.accel.timer.intervalf;
    auto& altitude = _model.state.altitude;
    const auto& altCfg = _model.config.altHold;

    if(_model.accelActive())
    {
      const float worldAccelZ = _accelFilter.update(_model.state.accel.world.z);
      const float linearAccelZ = worldAccelZ - altitude.accelBias;

      // always trim bias very slowly so integration never runs away; faster when on the ground
      const float biasGainGround = std::clamp((float)altCfg.accelBiasAlphaGround * 0.0001f, 0.0f, 0.05f);
      const float biasGainAir = std::clamp((float)altCfg.accelBiasAlphaAir * 0.0001f, 0.0f, 0.01f);
      const float biasGain = (!_model.isModeActive(MODE_ARMED) || _model.isThrottleLow()) ? biasGainGround : biasGainAir;
      altitude.accelBias += linearAccelZ * biasGain;

      altitude.accelVario += linearAccelZ * dt;
      altitude.accelHeight += altitude.accelVario * dt + 0.5f * linearAccelZ * dt * dt;
    }

    if(_model.baroActive())
    {
      altitude.baroHeight = _model.state.baro.altitudeGround;
      // continuous complementary correction every loop so accel drift can't accumulate
      const float baroPosWeight = std::clamp((float)altCfg.baroPosWeight * 0.01f, 0.0f, 1.0f);
      altitude.accelHeight += (altitude.baroHeight - altitude.accelHeight) * baroPosWeight;
      if(_model.state.baro.updateCount != altitude.baroUpdateCount)
      {
        altitude.baroUpdateCount = _model.state.baro.updateCount;
        const float baroVarioWeight = std::clamp((float)altCfg.baroVarioWeight * 0.01f, 0.0f, 1.0f);
        altitude.accelVario += (_model.state.baro.vario - altitude.accelVario) * baroVarioWeight;
      }
    }

    // Laser rangefinder gives an accurate surface distance close to the ground.
    // Only the dedicated MODE_SURFACE blends it in; the plain MODE_ALTHOLD stays
    // on baro + accel so behaviour matches the selected mode.
    if(_model.isModeActive(MODE_SURFACE) && _model.rangefinderActive() && _model.state.rangefinder.valid)
    {
      const float surfaceWeight = std::clamp((float)altCfg.surfaceWeight * 0.01f, 0.0f, 1.0f);
      altitude.accelHeight += (_model.state.rangefinder.height - altitude.accelHeight) * surfaceWeight;
    }

    altitude.accelVario = std::clamp(altitude.accelVario, -10.0f, 10.0f);

    altitude.height = altitude.accelHeight;
    altitude.vario = altitude.accelVario;

    if(_model.config.debug.mode == DEBUG_ALTITUDE)
    {
      _model.state.debug[0] = std::clamp(lrintf(altitude.baroHeight * 100.0f), -32000l, 32000l);               // baroAlt cm
      _model.state.debug[1] = std::clamp(lrintf(altitude.accelHeight * 100.0f), -32000l, 32000l);               // accelAlt cm
      _model.state.debug[2] = std::clamp(lrintf(altitude.height * 100.0f), -32000l, 32000l);                    // fusedAlt cm
      _model.state.debug[3] = std::clamp(lrintf(altitude.vario * 100.0f), -32000l, 32000l);                     // fusedVario cm/s
    }

    return 1;
  }

private:
  Model& _model;
  Utils::Filter _accelFilter;
};

}

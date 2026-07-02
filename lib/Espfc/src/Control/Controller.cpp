#include "Control/Controller.h"
#include "Utils/Math.hpp"
#include <algorithm>
#include <cmath>

namespace Espfc::Control {

static float getAltHoldNeutralThrottle(const Model& model)
{
  const auto& altCfg = model.config.altHold;
  const float hoverCenter = Utils::map((float)altCfg.itermCenter, 0.f, 100.f, -1.f, 1.f);

  switch (altCfg.throttleMode)
  {
    case ALTHOLD_THROTTLE_MID:
      return 0.0f;
    case ALTHOLD_THROTTLE_HOVER:
      return hoverCenter;
    case ALTHOLD_THROTTLE_STICK:
    default:
      // Match INAV behavior: use current stick when not low, fallback to mid-stick.
      return model.isThrottleLow() ? 0.0f : model.state.input.ch[AXIS_THRUST];
  }
}

Controller::Controller(Model& model): _model(model), _rates{}, _altHoldPrepareTakeoff(false) {}

int Controller::begin()
{
  _rates.begin(_model.config.input);
  _speedFilter.begin(FilterConfig(FILTER_BIQUAD, 10), _model.state.loopTimer.rate);

  beginInnerLoop(AXIS_ROLL);
  beginInnerLoop(AXIS_PITCH);
  beginInnerLoop(AXIS_YAW);
  beginOuterLoop(AXIS_ROLL);
  beginOuterLoop(AXIS_PITCH);
  beginAltHold();
  beginPosHold();

  return 1;
}

int FAST_CODE_ATTR Controller::update()
{
  uint32_t startTime = 0;
  if (_model.config.debug.mode == DEBUG_PIDLOOP)
  {
    startTime = micros();
    _model.state.debug[0] = startTime - _model.state.loopTimer.last;
  }

  {
    Utils::Stats::Measure(_model.state.stats, COUNTER_OUTER_PID);
    resetIterm();
    switch (_model.config.mixer.type)
    {
      case FC_MIXER_GIMBAL: outerLoopRobot(); break;

      default: outerLoop(); break;
    }
  }

  {
    Utils::Stats::Measure(_model.state.stats, COUNTER_INNER_PID);
    switch (_model.config.mixer.type)
    {
      case FC_MIXER_GIMBAL: innerLoopRobot(); break;

      default: innerLoop(); break;
    }
  }

  if (_model.config.debug.mode == DEBUG_PIDLOOP)
  {
    _model.state.debug[2] = micros() - startTime;
  }

  return 1;
}

void Controller::outerLoopRobot()
{
  const float speedScale = 2.f;
  const float gyroScale = 0.1f;
  const float speed = _speedFilter.update(_model.state.output.ch[AXIS_PITCH] * speedScale +
                                          _model.state.gyro.adc[AXIS_PITCH] * gyroScale);
  float angle = 0;
  const auto& input = _model.state.input;
  const auto& levelConf = _model.config.level;

  if (true || _model.isModeActive(MODE_ANGLE))
  {
    angle = input.ch[AXIS_PITCH] * Utils::toRad(levelConf.angleLimit);
  }
  else
  {
    angle = _model.state.outerPid[AXIS_PITCH].update(input.ch[AXIS_PITCH], speed) * Utils::toRad(levelConf.rateLimit);
  }
  _model.state.setpoint.angle.set(AXIS_PITCH, angle);
  _model.state.setpoint.rate[AXIS_YAW] = input.ch[AXIS_YAW] * Utils::toRad(levelConf.rateLimit);

  if (_model.config.debug.mode == DEBUG_ANGLERATE)
  {
    _model.state.debug[0] = speed * 1000;
    _model.state.debug[1] = lrintf(Utils::toDeg(angle) * 10);
  }
}

void Controller::innerLoopRobot()
{
  // VectorFloat v(0.f, 0.f, 1.f);
  // v.rotate(_model.state.attitude.quaternion);
  // const float angle = acos(v.z);

  const auto& attitude = _model.state.attitude;
  const auto& setpoint = _model.state.setpoint;

  auto& output = _model.state.output;
  auto& innerPid = _model.state.innerPid;

  const float angle = std::max(abs(attitude.euler[AXIS_PITCH]), abs(attitude.euler[AXIS_ROLL]));
  const bool stabilize = angle < Utils::toRad(_model.config.level.angleLimit);
  if (stabilize)
  {
    output.ch[AXIS_PITCH] = innerPid[AXIS_PITCH].update(setpoint.angle[AXIS_PITCH], attitude.euler[AXIS_PITCH]);
    output.ch[AXIS_YAW] = innerPid[AXIS_YAW].update(setpoint.rate[AXIS_YAW], _model.state.gyro.adc[AXIS_YAW]);
  }
  else
  {
    resetIterm();
    output.ch[AXIS_PITCH] = 0.f;
    output.ch[AXIS_YAW] = 0.f;
  }

  if (_model.config.debug.mode == DEBUG_ANGLERATE)
  {
    _model.state.debug[2] = lrintf(Utils::toDeg(attitude.euler[AXIS_PITCH]) * 10);
    _model.state.debug[3] = lrintf(output.ch[AXIS_PITCH] * 1000);
  }
}

void FAST_CODE_ATTR Controller::outerLoop()
{
  // optical-flow position hold produces auto lean-angle setpoints (angle mode only)
  updatePosHold();

  // Roll/Pitch rates control
  if (_model.isModeActive(MODE_ANGLE))
  {
    const float posHoldDeadband = _model.config.posHold.deadband * 0.01f;
    for (size_t i = 0; i < AXIS_COUNT_RP; i++)
    {
      float angleSetpoint;
      const bool centered = std::fabs(_model.state.input.ch[i]) < posHoldDeadband;
      if (_model.state.posHold.engaged && centered)
      {
        // stick centered while holding: fly to the auto position-hold lean angle
        angleSetpoint = _model.state.posHold.angle[i];
      }
      else
      {
        angleSetpoint = Utils::toRad(_model.config.level.angleLimit) * _model.state.input.ch[i];
      }
      _model.state.setpoint.rate[i] = _model.state.outerPid[i].update(angleSetpoint, _model.state.attitude.euler[i]);
      // disable fterm in angle mode
      _model.state.innerPid[i].fScale = 0.f;
    }
  }
  else
  {
    for (size_t i = 0; i < AXIS_COUNT_RP; i++)
    {
      _model.state.setpoint.rate[i] = calculateSetpointRate(i, _model.state.input.ch[i]);
    }
  }

  // Yaw rates control
  _model.state.setpoint.rate[AXIS_YAW] = calculateSetpointRate(AXIS_YAW, _model.state.input.ch[AXIS_YAW]);

  // thrust control: iNav-like. Alt-hold engages immediately and throttle stick
  // always commands vertical speed around the center deadband.
  if (_model.isAltHoldActive())
  {
    if (_model.hasChanged(MODE_ALTHOLD) || _model.hasChanged(MODE_SURFACE))
    {
      _model.state.altitude.target = _model.state.altitude.height;

      // Engage immediately on mode switch using configured ALTHOLD neutral throttle mode.
      _model.state.altitude.engaged = true;
      _model.state.altitude.hoverThrottle = getAltHoldNeutralThrottle(_model);

      _model.state.outerPid[AXIS_THRUST].resetIterm();
      _model.state.innerPid[AXIS_THRUST].resetIterm();

      // INAV-style ground engage handling: when enabling ALTHOLD at low throttle
      // and near the ground, pre-bias the throttle controller low to avoid a jump.
      _altHoldPrepareTakeoff = _model.isThrottleLow() && std::fabs(_model.state.altitude.height) <= 0.5f;
      if (_altHoldPrepareTakeoff)
      {
        const float maxManualClimbRate = std::clamp((float)_model.config.altHold.manualClimbRate, 10.f, 2000.f) * 0.01f;
        auto& velPid = _model.state.innerPid[AXIS_THRUST];
        velPid.iTerm = std::max(velPid.iLimitLow, -0.5f);

        auto& posPid = _model.state.outerPid[AXIS_THRUST];
        const float safeKp = std::max(posPid.Kp, 0.01f);
        _model.state.setpoint.rate[AXIS_THRUST] = -maxManualClimbRate;
        _model.state.altitude.target = _model.state.altitude.height - (maxManualClimbRate / safeKp);
      }
    }

    if (_altHoldPrepareTakeoff)
    {
      // iNav-like: while engaged on the ground, stay in takeoff-prepare until
      // pilot commands positive climb (stick above center deadband) or craft
      // has actually lifted.
      const float requestedClimbRate = calcualteAltHoldSetpoint();
      const bool armedForTakeoff = requestedClimbRate > 0.f || std::fabs(_model.state.altitude.height) > 0.5f;
      if (armedForTakeoff)
      {
        _altHoldPrepareTakeoff = false;
        _model.state.altitude.target = _model.state.altitude.height;
        _model.state.outerPid[AXIS_THRUST].resetIterm();
        _model.state.innerPid[AXIS_THRUST].resetIterm();
      }
      else
      {
        const float maxManualClimbRate = std::clamp((float)_model.config.altHold.manualClimbRate, 10.f, 2000.f) * 0.01f;
        _model.state.setpoint.rate[AXIS_THRUST] = -maxManualClimbRate;
        return;
      }
    }

    const float climbRate = calcualteAltHoldSetpoint();
    if (climbRate != 0.f)
    {
      // stick out of deadband: move the target and command the climb-rate directly
      _model.state.altitude.target += climbRate * _model.state.loopTimer.intervalf;
      _model.state.setpoint.rate[AXIS_THRUST] = climbRate;
    }
    else
    {
      // stick centered: hold target altitude, position PID derives the climb-rate
      _model.state.setpoint.rate[AXIS_THRUST] = _model.state.outerPid[AXIS_THRUST].update(_model.state.altitude.target, _model.state.altitude.height);
    }
  }
  else
  {
    _altHoldPrepareTakeoff = false;
    if (_model.hasChanged(MODE_ALTHOLD) || _model.hasChanged(MODE_SURFACE))
    {
      _model.state.outerPid[AXIS_THRUST].resetIterm();
      _model.state.innerPid[AXIS_THRUST].resetIterm();
    }
    _model.state.altitude.target = _model.state.altitude.height;
    _model.state.altitude.engaged = false;
    _model.state.setpoint.rate[AXIS_THRUST] = _model.state.input.ch[AXIS_THRUST];
  }

  // debug
  if (_model.config.debug.mode == DEBUG_ANGLERATE)
  {
    for (size_t i = 0; i < AXIS_COUNT_RPY; ++i)
    {
      _model.state.debug[i] = lrintf(Utils::toDeg(_model.state.setpoint.rate[i]));
    }
  }
}

void FAST_CODE_ATTR Controller::innerLoop()
{
  // Roll/Pitch/Yaw rates control
  const float tpaFactor = getTpaFactor();
  const auto& setpoint = _model.state.setpoint;

  auto& innerPid = _model.state.innerPid;
  auto& output = _model.state.output;

  for (size_t i = 0; i < AXIS_COUNT_RPY; ++i)
  {
    output.ch[i] = innerPid[i].update(setpoint.rate[i], _model.state.gyro.adc[i]) * tpaFactor;
  }

  // thrust control: before engage -> manual stick (sits on the ground);
  // after engage -> hover baseline + vel PID climb-rate trim
  if (_model.isAltHoldActive() && _model.state.altitude.engaged)
  {
    const float minThrust = std::clamp(Utils::map((float)_model.config.output.minThrottle, 1000.f, 2000.f, -1.f, 1.f), -1.f, 1.f);
    const float maxThrust = 1.0f;
    const float hoverThrottle = std::clamp(_model.state.altitude.hoverThrottle, minThrust, maxThrust);

    // Dynamic correction bounds improve anti-windup near thrust limits.
    innerPid[AXIS_THRUST].oLimitLow = minThrust - hoverThrottle;
    innerPid[AXIS_THRUST].oLimitHigh = maxThrust - hoverThrottle;

    const float correction = innerPid[AXIS_THRUST].update(setpoint.rate[AXIS_THRUST], _model.state.altitude.vario);
    output.ch[AXIS_THRUST] = std::clamp(hoverThrottle + correction, minThrust, maxThrust);
  }
  else
  {
    innerPid[AXIS_THRUST].resetIterm();
    innerPid[AXIS_THRUST].oLimitLow = -0.5f;
    innerPid[AXIS_THRUST].oLimitHigh = 0.5f;
    output.ch[AXIS_THRUST] = _model.state.input.ch[AXIS_THRUST];
  }

  if (_model.config.debug.mode == DEBUG_STACK)
  {
    _model.state.debug[0] = std::clamp(lrintf(setpoint.rate[AXIS_THRUST] * 1000.0f), -3000l, 3000l);    // hi mem
    _model.state.debug[1] = std::clamp(lrintf(_model.state.altitude.vario * 1000.0f), -30000l, 30000l); // lo mem
    _model.state.debug[2] = std::clamp(lrintf(_model.state.altitude.height * 100.0f), -30000l, 30000l); // curr
    _model.state.debug[3] = std::clamp(lrintf(innerPid[AXIS_THRUST].error * 1000.0f), -30000l, 30000l); // p
    _model.state.debug[4] = std::clamp(lrintf(innerPid[AXIS_THRUST].pTerm * 1000.0f), -3000l, 3000l);
    _model.state.debug[5] = std::clamp(lrintf(innerPid[AXIS_THRUST].iTerm * 1000.0f), -3000l, 3000l);
    _model.state.debug[6] = std::clamp(lrintf(innerPid[AXIS_THRUST].dTerm * 1000.0f), -3000l, 3000l);
    _model.state.debug[7] = std::clamp(lrintf(innerPid[AXIS_THRUST].fTerm * 1000.0f), -3000l, 3000l);
  }

  // debug
  if (_model.config.debug.mode == DEBUG_ITERM_RELAX)
  {
    _model.state.debug[0] = lrintf(Utils::toDeg(innerPid[AXIS_ROLL].itermRelaxBase));
    _model.state.debug[1] = lrintf(innerPid[AXIS_ROLL].itermRelaxFactor * 100.0f);
    _model.state.debug[2] = lrintf(Utils::toDeg(innerPid[AXIS_ROLL].iTermError));
    _model.state.debug[3] = lrintf(innerPid[AXIS_ROLL].iTerm * 1000.0f);
  }
}

float Controller::calcualteAltHoldSetpoint() const
{
  float thrust = _model.state.input.ch[AXIS_THRUST]; // -1..1, 0 = center

  const float deadband = std::clamp((float)_model.config.altHold.stickDeadband * 0.01f, 0.01f, 0.45f);
  thrust = Utils::deadband(thrust, deadband);
  if (thrust == 0.f) return 0.f;

  const float span = 1.0f - deadband;
  const float climbRateNorm = Utils::map3(thrust, -span, 0.f, span, -1.0f, 0.f, 1.0f);

  // Symmetric manual climb command around neutral, scaled by configured max climb rate.
  const float maxManualClimbRate = std::clamp((float)_model.config.altHold.manualClimbRate, 10.f, 2000.f) * 0.01f; // cm/s -> m/s
  return climbRateNorm * maxManualClimbRate;
}

float Controller::getTpaFactor() const
{
  if (_model.config.controller.tpaScale == 0) return 1.f;
  float t = Utils::clamp(_model.state.input.us[AXIS_THRUST], (float)_model.config.controller.tpaBreakpoint, 2000.f);
  return Utils::map(t, (float)_model.config.controller.tpaBreakpoint, 2000.f, 1.f,
                    1.f - ((float)_model.config.controller.tpaScale * 0.01f));
}

void Controller::resetIterm()
{
  if (!_model.isModeActive(MODE_ARMED) // when not armed
      || (!_model.isAirModeActive() && _model.config.iterm.lowThrottleZeroIterm &&
          _model.isThrottleLow()) // on low throttle (not in air mode)
  )
  {
    for (size_t i = 0; i < AXIS_COUNT_RPY; i++)
    {
      _model.state.innerPid[i].resetIterm();
      _model.state.outerPid[i].resetIterm();
    }
  }
  if (!_model.isModeActive(MODE_ARMED))
  {
    //_model.state.innerPid[AXIS_THRUST].resetIterm();
  }
}

float Controller::calculateSetpointRate(int axis, float input) const
{
  if (axis == AXIS_YAW) input *= -1.f;
  return _rates.getSetpoint(axis, input);
}

void Controller::beginInnerLoop(size_t axis)
{
  const int pidFilterRate = _model.state.loopTimer.rate;
  float pidScale[] = {1.f, 1.f, 1.f};
  if (_model.config.mixer.type == FC_MIXER_GIMBAL)
  {
    pidScale[AXIS_YAW] = 0.2f;   // ROBOT
    pidScale[AXIS_PITCH] = 20.f; // ROBOT
  }

  const auto& pc = _model.config.pid[axis];
  const auto& dtermConf = _model.config.dterm;

  auto& pid = _model.state.innerPid[axis];
  pid.Kp = (float)pc.P * PTERM_SCALE * pidScale[axis];
  pid.Ki = (float)pc.I * ITERM_SCALE * pidScale[axis];
  pid.Kd = (float)pc.D * DTERM_SCALE * pidScale[axis];
  pid.Kf = (float)pc.F * FTERM_SCALE * pidScale[axis];
  pid.iLimitLow = -_model.config.iterm.limit * 0.01f;
  pid.iLimitHigh = _model.config.iterm.limit * 0.01f;
  pid.oLimitLow = -0.66f;
  pid.oLimitHigh = 0.66f;
  pid.rate = pidFilterRate;
  pid.dtermNotchFilter.begin(dtermConf.notchFilter, pidFilterRate);
  if (dtermConf.dynLpfFilter.cutoff > 0)
  {
    pid.dtermFilter.begin(FilterConfig((FilterType)dtermConf.filter.type, dtermConf.dynLpfFilter.cutoff),
                          pidFilterRate);
  }
  else
  {
    pid.dtermFilter.begin(dtermConf.filter, pidFilterRate);
  }
  pid.dtermFilter2.begin(dtermConf.filter2, pidFilterRate);
  pid.ftermFilter.begin(_model.config.input.filterDerivative, pidFilterRate);
  pid.itermRelaxFilter.begin(FilterConfig(FILTER_PT1, _model.config.iterm.relaxCutoff), pidFilterRate);
  if (axis == AXIS_YAW)
  {
    pid.itermRelax = (_model.config.iterm.relax == ITERM_RELAX_RPY || _model.config.iterm.relax == ITERM_RELAX_RPY_INC)
                         ? _model.config.iterm.relax
                         : ITERM_RELAX_OFF;
    pid.ptermFilter.begin(_model.config.yaw.filter, pidFilterRate);
  }
  else
  {
    pid.itermRelax = _model.config.iterm.relax;
  }
  pid.begin();
}

void Controller::beginOuterLoop(size_t axis)
{
  const int pidFilterRate = _model.state.loopTimer.rate;
  const auto& pc = _model.config.pid[FC_PID_LEVEL];

  auto& pid = _model.state.outerPid[axis];
  pid.Kp = (float)pc.P * LEVEL_PTERM_SCALE;
  pid.Ki = (float)pc.I * LEVEL_ITERM_SCALE;
  pid.Kd = (float)pc.D * LEVEL_DTERM_SCALE;
  pid.Kf = (float)pc.F * LEVEL_FTERM_SCALE;
  pid.iLimitHigh = Utils::toRad(_model.config.level.rateLimit * 0.1f);
  pid.iLimitLow = -pid.iLimitHigh;
  pid.oLimitHigh = Utils::toRad(_model.config.level.rateLimit);
  pid.oLimitLow = -pid.oLimitHigh;
  pid.rate = pidFilterRate;
  pid.ptermFilter.begin(_model.config.level.ptermFilter, pidFilterRate);
  // pid.iLimit = 0.3f; // ROBOT
  // pid.oLimit = 1.f;  // ROBOT
  pid.begin();
}

void Controller::beginAltHold()
{
  const auto& pcAlt = _model.config.pid[FC_PID_ALT];

  auto& posPid = _model.state.outerPid[AXIS_THRUST];
  posPid.Kp = (float)pcAlt.P * 0.1f;
  posPid.Ki = (float)pcAlt.I * 0.01f;
  posPid.Kd = (float)pcAlt.D * 0.001f;
  posPid.Kf = (float)pcAlt.F * 0.001f;
  posPid.iLimitLow = -1.0f;
  posPid.iLimitHigh = 1.0f;
  posPid.iReset = 0.0f;
  posPid.oLimitLow = -2.0f;
  posPid.oLimitHigh = 4.0f;
  posPid.rate = _model.state.loopTimer.rate;
  posPid.ptermFilter.begin(FilterConfig(FILTER_PT1, 5), _model.state.loopTimer.rate);
  posPid.dtermFilter.begin(FilterConfig(FILTER_PT1, 5), _model.state.loopTimer.rate);
  posPid.ftermDerivative = false;
  posPid.begin();

  const auto& pc = _model.config.pid[FC_PID_VEL];

  // vel pid produces a throttle correction around hover baseline (range roughly -0.6..0.6)
  auto& pid = _model.state.innerPid[AXIS_THRUST];
  pid.Kp = (float)pc.P * VEL_PTERM_SCALE;
  pid.Ki = (float)pc.I * VEL_ITERM_SCALE;
  pid.Kd = (float)pc.D * VEL_DTERM_SCALE;
  pid.Kf = (float)pc.F * VEL_FTERM_SCALE;
  pid.iLimitLow = -0.4f;
  pid.iLimitHigh = 0.4f;
  pid.iReset = 0.0f;
  pid.oLimitLow = -0.5f;
  pid.oLimitHigh = 0.5f;
  pid.rate = _model.state.loopTimer.rate;
  pid.dtermFilter.begin(FilterConfig(FILTER_PT1, 10), _model.state.loopTimer.rate);
  pid.ftermDerivative = false;
  pid.begin();
}

void Controller::beginPosHold()
{
  auto& ph = _model.state.posHold;
  const int rate = _model.state.loopTimer.rate;
  const float angleLimitRad = Utils::toRad((float)_model.config.posHold.angleLimit);

  const auto& pcPos = _model.config.pid[FC_PID_POS];
  Control::Pid* posPids[] = { &ph.pidPosX, &ph.pidPosY };
  for (auto* p : posPids)
  {
    // position error [m] -> desired ground velocity [m/s]
    p->Kp = (float)pcPos.P * 0.02f;
    p->Ki = (float)pcPos.I * 0.002f;
    p->Kd = 0.0f;
    p->Kf = 0.0f;
    p->iLimitLow = -1.0f;
    p->iLimitHigh = 1.0f;
    p->iReset = 0.0f;
    p->oLimitLow = -1.5f;
    p->oLimitHigh = 1.5f;
    p->rate = rate;
    p->ptermFilter.begin(FilterConfig(FILTER_PT1, 10), rate);
    p->dtermFilter.begin(FilterConfig(FILTER_PT1, 10), rate);
    p->ftermDerivative = false;
    p->begin();
  }

  const auto& pcVel = _model.config.pid[FC_PID_POSR];
  Control::Pid* velPids[] = { &ph.pidVelX, &ph.pidVelY };
  for (auto* p : velPids)
  {
    // ground velocity [m/s] -> lean angle [rad]
    p->Kp = (float)pcVel.P * 0.006f;
    p->Ki = (float)pcVel.I * 0.002f;
    p->Kd = (float)pcVel.D * 0.002f;
    p->Kf = 0.0f;
    p->iLimitLow = -angleLimitRad;
    p->iLimitHigh = angleLimitRad;
    p->iReset = 0.0f;
    p->oLimitLow = -angleLimitRad;
    p->oLimitHigh = angleLimitRad;
    p->rate = rate;
    p->ptermFilter.begin(FilterConfig(FILTER_PT1, 20), rate);
    p->dtermFilter.begin(FilterConfig(FILTER_PT1, 20), rate);
    p->ftermDerivative = false;
    p->begin();
  }

  ph.velFilterX.begin(_model.config.posHold.velFilter, 50);
  ph.velFilterY.begin(_model.config.posHold.velFilter, 50);
}

void FAST_CODE_ATTR Controller::updatePosHold()
{
  auto& ph = _model.state.posHold;
  const auto& cfg = _model.config.posHold;

  // Keep optical-flow status fresh, similar to rangefinder MSP timeout handling.
  constexpr uint32_t FLOW_TIMEOUT_US = 200000; // 200 ms
  const uint32_t flowAgeUs = (_model.state.flow.lastMsgTs == 0)
      ? FLOW_TIMEOUT_US + 1
      : (uint32_t)(micros() - _model.state.flow.lastMsgTs);
  const bool flowFresh = flowAgeUs <= FLOW_TIMEOUT_US;
  if (!flowFresh)
  {
    _model.state.flow.present = false;
    _model.state.flow.valid = false;
  }

  const bool active = _model.isPosHoldActive()
      && _model.isModeActive(MODE_ANGLE)
      && _model.isModeActive(MODE_ARMED)
      && flowFresh
      && _model.state.flow.valid
      && _model.rangefinderActive()
      && _model.state.rangefinder.valid
      && _model.state.rangefinder.height > 0.05f;

  if (!active)
  {
    ph.engaged = false;
    ph.posX = ph.posY = 0.0f;
    ph.angle[0] = ph.angle[1] = 0.0f;
    ph.pidPosX.iTerm = ph.pidPosY.iTerm = 0.0f;
    ph.pidVelX.iTerm = ph.pidVelY.iTerm = 0.0f;
    return;
  }

  // refresh the ground-velocity estimate on every new optical-flow sample
  if (_model.state.flow.lastMsgTs != ph.lastFlowTs)
  {
    float dtFlow = (ph.lastFlowTs == 0) ? 0.02f : (_model.state.flow.lastMsgTs - ph.lastFlowTs) * 1e-6f;
    dtFlow = std::clamp(dtFlow, 0.005f, 0.1f);
    ph.lastFlowTs = _model.state.flow.lastMsgTs;

    // motion counts -> angular rate [rad/s], sign/scale set by flowGain (orientation calibration)
    float rateX = _model.state.flow.motionX * (cfg.flowGainX * 1e-4f) / dtFlow;
    float rateY = _model.state.flow.motionY * (cfg.flowGainY * 1e-4f) / dtFlow;
    if (cfg.useGyroComp)
    {
      // remove the flow induced purely by the craft rotating in place
      rateX -= _model.state.gyro.adc[AXIS_ROLL];
      rateY -= _model.state.gyro.adc[AXIS_PITCH];
    }
    const float height = std::max(_model.state.rangefinder.height, 0.05f);
    ph.velX = ph.velFilterX.update(rateX * height);
    ph.velY = ph.velFilterY.update(rateY * height);
  }

  ph.engaged = true;

  const float dt = _model.state.loopTimer.intervalf;
  const float deadband = cfg.deadband * 0.01f;
  const bool centeredRoll = std::fabs(_model.state.input.ch[AXIS_ROLL]) < deadband;
  const bool centeredPitch = std::fabs(_model.state.input.ch[AXIS_PITCH]) < deadband;

  // body X (roll axis)
  if (centeredRoll)
  {
    ph.posX += ph.velX * dt;
    const float velSp = ph.pidPosX.update(0.0f, ph.posX);
    ph.angle[AXIS_ROLL] = ph.pidVelX.update(velSp, ph.velX);
  }
  else
  {
    // pilot commands roll: release the anchor and let them fly
    ph.posX = 0.0f;
    ph.pidPosX.iTerm = 0.0f;
    ph.pidVelX.iTerm = 0.0f;
    ph.angle[AXIS_ROLL] = 0.0f;
  }

  // body Y (pitch axis)
  if (centeredPitch)
  {
    ph.posY += ph.velY * dt;
    const float velSp = ph.pidPosY.update(0.0f, ph.posY);
    ph.angle[AXIS_PITCH] = ph.pidVelY.update(velSp, ph.velY);
  }
  else
  {
    ph.posY = 0.0f;
    ph.pidPosY.iTerm = 0.0f;
    ph.pidVelY.iTerm = 0.0f;
    ph.angle[AXIS_PITCH] = 0.0f;
  }
}

} // namespace Espfc::Control

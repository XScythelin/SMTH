#include "RangefinderVL53L0X.hpp"

// I2C address
#define VL53L0X_ADDRESS 0x29

// Registers
#define VL53L0X_SYSRANGE_START 0x00
#define VL53L0X_SYSTEM_SEQUENCE_CONFIG 0x01
#define VL53L0X_SYSTEM_INTERRUPT_CONFIG_GPIO 0x0A
#define VL53L0X_SYSTEM_INTERRUPT_CLEAR 0x0B
#define VL53L0X_RESULT_INTERRUPT_STATUS 0x13
#define VL53L0X_RESULT_RANGE_STATUS 0x14
#define VL53L0X_MSRC_CONFIG_CONTROL 0x60
#define VL53L0X_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT 0x44
#define VL53L0X_GLOBAL_CONFIG_SPAD_ENABLES_REF_0 0xB0
#define VL53L0X_DYNAMIC_SPAD_REF_EN_START_OFFSET 0x4F
#define VL53L0X_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD 0x4E
#define VL53L0X_GLOBAL_CONFIG_REF_EN_START_SELECT 0xB6
#define VL53L0X_GPIO_HV_MUX_ACTIVE_HIGH 0x84
#define VL53L0X_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV 0x89
#define VL53L0X_IDENTIFICATION_MODEL_ID 0xC0
#define VL53L0X_MODEL_ID 0xEE

#define VL53L0X_IO_TIMEOUT_MS 50u

namespace Espfc::Device::Rangefinder {

uint8_t RangefinderVL53L0X::readReg(uint8_t reg)
{
  uint8_t v = 0;
  _bus->read(_addr, reg, 1, &v);
  return v;
}

uint16_t RangefinderVL53L0X::readReg16(uint8_t reg)
{
  uint8_t b[2] = {0, 0};
  _bus->read(_addr, reg, 2, b);
  return ((uint16_t)b[0] << 8) | b[1];
}

void RangefinderVL53L0X::readMulti(uint8_t reg, uint8_t* dst, uint8_t count)
{
  _bus->read(_addr, reg, count, dst);
}

void RangefinderVL53L0X::writeReg(uint8_t reg, uint8_t val)
{
  _bus->write(_addr, reg, 1, &val);
}

void RangefinderVL53L0X::writeReg16(uint8_t reg, uint16_t val)
{
  uint8_t b[2] = {(uint8_t)(val >> 8), (uint8_t)(val & 0xFF)};
  _bus->write(_addr, reg, 2, b);
}

void RangefinderVL53L0X::writeMulti(uint8_t reg, const uint8_t* src, uint8_t count)
{
  _bus->write(_addr, reg, count, src);
}

bool RangefinderVL53L0X::testConnection()
{
  return readReg(VL53L0X_IDENTIFICATION_MODEL_ID) == VL53L0X_MODEL_ID;
}

int RangefinderVL53L0X::begin(BusDevice* bus)
{
  return begin(bus, VL53L0X_ADDRESS);
}

int RangefinderVL53L0X::begin(BusDevice* bus, uint8_t addr)
{
  setBus(bus, addr);

  if (!testConnection()) return 0;

  // --- Data init: switch sensor I/O to 2.8V mode ---
  writeReg(VL53L0X_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV, readReg(VL53L0X_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV) | 0x01);

  // set I2C standard mode
  writeReg(0x88, 0x00);

  writeReg(0x80, 0x01);
  writeReg(0xFF, 0x01);
  writeReg(0x00, 0x00);
  _stopVariable = readReg(0x91);
  writeReg(0x00, 0x01);
  writeReg(0xFF, 0x00);
  writeReg(0x80, 0x00);

  // disable SIGNAL_RATE_MSRC and SIGNAL_RATE_PRE_RANGE limit checks
  writeReg(VL53L0X_MSRC_CONFIG_CONTROL, readReg(VL53L0X_MSRC_CONFIG_CONTROL) | 0x12);

  // set final range signal rate limit to 0.25 MCPS (million counts per second)
  writeReg16(VL53L0X_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, (uint16_t)(0.25f * (1 << 7)));

  writeReg(VL53L0X_SYSTEM_SEQUENCE_CONFIG, 0xFF);

  // --- Static init: configure SPADs ---
  uint8_t spadCount = 0;
  bool spadTypeIsAperture = false;
  if (!getSpadInfo(&spadCount, &spadTypeIsAperture)) return 0;

  uint8_t refSpadMap[6] = {0, 0, 0, 0, 0, 0};
  readMulti(VL53L0X_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, refSpadMap, 6);

  writeReg(0xFF, 0x01);
  writeReg(VL53L0X_DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
  writeReg(VL53L0X_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
  writeReg(0xFF, 0x00);
  writeReg(VL53L0X_GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);

  const uint8_t firstSpadToEnable = spadTypeIsAperture ? 12 : 0; // 12 is the first aperture spad
  uint8_t spadsEnabled = 0;
  for (uint8_t i = 0; i < 48; i++)
  {
    if (i < firstSpadToEnable || spadsEnabled == spadCount)
    {
      refSpadMap[i / 8] &= ~(1 << (i % 8));
    }
    else if ((refSpadMap[i / 8] >> (i % 8)) & 0x1)
    {
      spadsEnabled++;
    }
  }
  writeMulti(VL53L0X_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, refSpadMap, 6);

  // --- Load default tuning settings ---
  static const uint8_t tuning[] = {
    0xFF, 0x01, 0x00, 0x00, 0xFF, 0x00, 0x09, 0x00, 0x10, 0x00, 0x11, 0x00,
    0x24, 0x01, 0x25, 0xFF, 0x75, 0x00, 0xFF, 0x01, 0x4E, 0x2C, 0x48, 0x00,
    0x30, 0x20, 0xFF, 0x00, 0x30, 0x09, 0x54, 0x00, 0x31, 0x04, 0x32, 0x03,
    0x40, 0x83, 0x46, 0x25, 0x60, 0x00, 0x27, 0x00, 0x50, 0x06, 0x51, 0x00,
    0x52, 0x96, 0x56, 0x08, 0x57, 0x30, 0x61, 0x00, 0x62, 0x00, 0x64, 0x00,
    0x65, 0x00, 0x66, 0xA0, 0xFF, 0x01, 0x22, 0x32, 0x47, 0x14, 0x49, 0xFF,
    0x4A, 0x00, 0xFF, 0x00, 0x7A, 0x0A, 0x7B, 0x00, 0x78, 0x21, 0xFF, 0x01,
    0x23, 0x34, 0x42, 0x00, 0x44, 0xFF, 0x45, 0x26, 0x46, 0x05, 0x40, 0x40,
    0x0E, 0x06, 0x20, 0x1A, 0x43, 0x40, 0xFF, 0x00, 0x34, 0x03, 0x35, 0x44,
    0xFF, 0x01, 0x31, 0x04, 0x4B, 0x09, 0x4C, 0x05, 0x4D, 0x04, 0xFF, 0x00,
    0x44, 0x00, 0x45, 0x20, 0x47, 0x08, 0x48, 0x28, 0x67, 0x00, 0x70, 0x04,
    0x71, 0x01, 0x72, 0xFE, 0x76, 0x00, 0x77, 0x00, 0xFF, 0x01, 0x0D, 0x01,
    0xFF, 0x00, 0x80, 0x01, 0x01, 0xF8, 0xFF, 0x01, 0x8E, 0x01, 0x00, 0x01,
    0xFF, 0x00, 0x80, 0x00,
  };
  for (size_t i = 0; i < sizeof(tuning); i += 2)
  {
    writeReg(tuning[i], tuning[i + 1]);
  }

  // --- Configure interrupt to signal "new sample ready" ---
  writeReg(VL53L0X_SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
  writeReg(VL53L0X_GPIO_HV_MUX_ACTIVE_HIGH, readReg(VL53L0X_GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10); // active low
  writeReg(VL53L0X_SYSTEM_INTERRUPT_CLEAR, 0x01);

  writeReg(VL53L0X_SYSTEM_SEQUENCE_CONFIG, 0xE8);

  // --- Reference calibration ---
  writeReg(VL53L0X_SYSTEM_SEQUENCE_CONFIG, 0x01);
  if (!performSingleRefCalibration(0x40)) return 0; // VHV
  writeReg(VL53L0X_SYSTEM_SEQUENCE_CONFIG, 0x02);
  if (!performSingleRefCalibration(0x00)) return 0; // Phase
  writeReg(VL53L0X_SYSTEM_SEQUENCE_CONFIG, 0xE8); // restore previous sequence config

  // --- Start continuous back-to-back ranging ---
  writeReg(0x80, 0x01);
  writeReg(0xFF, 0x01);
  writeReg(0x00, 0x00);
  writeReg(0x91, _stopVariable);
  writeReg(0x00, 0x01);
  writeReg(0xFF, 0x00);
  writeReg(0x80, 0x00);
  writeReg(VL53L0X_SYSRANGE_START, 0x02); // back-to-back mode

  return 1;
}

bool RangefinderVL53L0X::getSpadInfo(uint8_t* count, bool* typeIsAperture)
{
  writeReg(0x80, 0x01);
  writeReg(0xFF, 0x01);
  writeReg(0x00, 0x00);

  writeReg(0xFF, 0x06);
  writeReg(0x83, readReg(0x83) | 0x04);
  writeReg(0xFF, 0x07);
  writeReg(0x81, 0x01);

  writeReg(0x80, 0x01);

  writeReg(0x94, 0x6B);
  writeReg(0x83, 0x00);

  uint32_t start = millis();
  while (readReg(0x83) == 0x00)
  {
    if (millis() - start > VL53L0X_IO_TIMEOUT_MS) return false;
  }
  writeReg(0x83, 0x01);
  uint8_t tmp = readReg(0x92);

  *count = tmp & 0x7F;
  *typeIsAperture = (tmp >> 7) & 0x01;

  writeReg(0x81, 0x00);
  writeReg(0xFF, 0x06);
  writeReg(0x83, readReg(0x83) & ~0x04);
  writeReg(0xFF, 0x01);
  writeReg(0x00, 0x01);

  writeReg(0xFF, 0x00);
  writeReg(0x80, 0x00);

  return true;
}

bool RangefinderVL53L0X::performSingleRefCalibration(uint8_t vhvInitByte)
{
  writeReg(VL53L0X_SYSRANGE_START, 0x01 | vhvInitByte);

  uint32_t start = millis();
  while ((readReg(VL53L0X_RESULT_INTERRUPT_STATUS) & 0x07) == 0)
  {
    if (millis() - start > VL53L0X_IO_TIMEOUT_MS) return false;
  }

  writeReg(VL53L0X_SYSTEM_INTERRUPT_CLEAR, 0x01);
  writeReg(VL53L0X_SYSRANGE_START, 0x00);

  return true;
}

RangefinderDeviceType RangefinderVL53L0X::getType() const
{
  return RANGEFINDER_VL53L0X;
}

int32_t RangefinderVL53L0X::readRangeMm()
{
  // non-blocking: bail out until a new sample is ready
  if ((readReg(VL53L0X_RESULT_INTERRUPT_STATUS) & 0x07) == 0) return -1;

  // range is at RESULT_RANGE_STATUS + 10
  uint16_t range = readReg16(VL53L0X_RESULT_RANGE_STATUS + 10);
  writeReg(VL53L0X_SYSTEM_INTERRUPT_CLEAR, 0x01);

  // 8190 / 8191 are returned when no target is detected (out of range)
  if (range >= 8190) return -1;

  return range;
}

int RangefinderVL53L0X::getDelay() const
{
  return 33000; // ~30 Hz default measurement timing budget
}

int32_t RangefinderVL53L0X::getMaxRangeMm() const
{
  return 2000; // ~2 m reliable range
}

} // namespace Espfc::Device::Rangefinder

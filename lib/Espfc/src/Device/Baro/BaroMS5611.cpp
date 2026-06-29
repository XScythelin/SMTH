#include "BaroMS5611.hpp"
#include <Arduino.h>

// MS5611-01BA03 I2C addresses (CSB pin selects)
#define MS5611_ADDRESS_FIRST  0x76
#define MS5611_ADDRESS_SECOND 0x77

// Commands
#define MS5611_CMD_RESET      0x1E
#define MS5611_CMD_CONVERT_D1 0x48  // Convert pressure,    OSR=4096
#define MS5611_CMD_CONVERT_D2 0x58  // Convert temperature, OSR=4096
#define MS5611_CMD_ADC_READ   0x00
#define MS5611_CMD_PROM_BASE  0xA0  // PROM word i at (0xA0 + i*2)

// Some MS5611 clones are slower than datasheet typical timings on ESP32 I2C.
// 12ms matches the known-good standalone sketch behavior.
#define MS5611_CONVERSION_TIME_US 12000

namespace Espfc::Device::Baro {

// MS5611 is not a register-based device. It uses single-byte commands over I2C,
// and does NOT ACK during ADC conversion (~10ms). BusI2C::write/read always call
// endTransmission which triggers onError on NAK. Since this is normal MS5611
// behavior (not a real bus error), we suppress onError for all MS5611 transactions.

// RAII guard: suppresses bus onError for the lifetime of the object
struct SuppressI2CError {
  SuppressI2CError(BusDevice* bus) : _bus(bus), _saved(bus->onError) {
    _bus->onError = nullptr;
  }
  ~SuppressI2CError() {
    _bus->onError = _saved;
  }
  BusDevice* _bus;
  std::function<void(void)> _saved;
};

int BaroMS5611::begin(BusDevice* bus)
{
  return begin(bus, MS5611_ADDRESS_FIRST) ? 1 : begin(bus, MS5611_ADDRESS_SECOND) ? 1 : 0;
}

int BaroMS5611::begin(BusDevice* bus, uint8_t addr)
{
  setBus(bus, addr);

  sendCommand(MS5611_CMD_RESET);
  delay(10); // datasheet min 2.8ms; use 10ms for reliable PROM read on all chips

  if (!testConnection()) return 0;

  return 1;
}

BaroDeviceType BaroMS5611::getType() const
{
  return BARO_MS5611;
}

float BaroMS5611::readTemperature()
{
  uint32_t D2 = readADC();
  if (D2 == 0) return _t_fine * 0.01f;

  _dT = (int32_t)D2 - ((int32_t)_c[4] << 8);
  int32_t TEMP = 2000 + (int32_t)(((int64_t)_dT * _c[5]) >> 23);

  if (TEMP < 2000)
  {
    int32_t T2 = (int32_t)(((int64_t)_dT * _dT) >> 31);
    TEMP -= T2;
  }

  _t_fine = TEMP;
  return TEMP * 0.01f;
}

float BaroMS5611::readPressure()
{
  uint32_t D1 = readADC();
  if (D1 == 0) return _pressure;

  int32_t TEMP = _t_fine;

  int64_t OFF  = ((int64_t)_c[1] << 16) + (((int64_t)_c[3] * _dT) >> 7);
  int64_t SENS = ((int64_t)_c[0] << 15) + (((int64_t)_c[2] * _dT) >> 8);

  if (TEMP < 2000)
  {
    int32_t tmp   = (TEMP - 2000) * (TEMP - 2000);
    int64_t OFF2  = (5LL * tmp) >> 1;
    int64_t SENS2 = (5LL * tmp) >> 2;
    if (TEMP < -1500)
    {
      int32_t tmp2 = (TEMP + 1500) * (TEMP + 1500);
      OFF2  += 7LL * tmp2;
      SENS2 += (11LL * tmp2) >> 1;
    }
    OFF  -= OFF2;
    SENS -= SENS2;
  }

  int64_t P = ((int64_t)D1 * SENS >> 21) - OFF;
  P >>= 15;

  _pressure = (float)(int32_t)P;
  return _pressure;
}

void BaroMS5611::setMode(BaroDeviceMode mode)
{
  if (mode == BARO_MODE_TEMP)
    sendCommand(MS5611_CMD_CONVERT_D2);
  else
    sendCommand(MS5611_CMD_CONVERT_D1);
}

int BaroMS5611::getDelay(BaroDeviceMode mode) const
{
  (void)mode;
  return MS5611_CONVERSION_TIME_US;
}

bool BaroMS5611::testConnection()
{
  return readPROM();
}

// MS5611 sends a bare command byte — no register address, no payload.
// NAK during conversion is normal; suppress onError.
int8_t BaroMS5611::sendCommand(uint8_t cmd)
{
  SuppressI2CError guard(_bus);
  return _bus->write(_addr, cmd, 0, nullptr);
}

// ADC read: send 0x00 then read 3 bytes.
// The write phase (sending 0x00) may NAK if chip is still converting.
uint32_t BaroMS5611::readADC()
{
  SuppressI2CError guard(_bus);
  uint8_t buf[3] = {0, 0, 0};
  if (_bus->read(_addr, MS5611_CMD_ADC_READ, 3, buf) != 3) return 0;
  return ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];
}

// PROM read is normal register-style access — no suppression needed here,
// but keep guard for consistency since chip just came out of reset.
bool BaroMS5611::readPROM()
{
  SuppressI2CError guard(_bus);
  uint16_t prom[8];
  for (int i = 0; i < 8; i++)
  {
    uint8_t buf[2] = {0, 0};
    if (_bus->read(_addr, MS5611_CMD_PROM_BASE + i * 2, 2, buf) != 2) return false;
    prom[i] = ((uint16_t)buf[0] << 8) | buf[1];
  }

  bool anyCoef = false;
  for (int i = 1; i <= 6; i++)
  {
    if (prom[i] != 0 && prom[i] != 0xFFFF)
    {
      anyCoef = true;
      break;
    }
  }
  if (!anyCoef) return false;

  uint8_t crcExpected = prom[7] & 0x000F;
  if (crc4(prom) != crcExpected) return false;

  for (int i = 0; i < 6; i++)
    _c[i] = prom[i + 1];

  return true;
}

uint8_t BaroMS5611::crc4(uint16_t prom[8])
{
  uint16_t n_rem  = 0;
  uint16_t saved  = prom[7];
  prom[7]        &= 0xFF00;

  for (int cnt = 0; cnt < 16; cnt++)
  {
    if (cnt % 2 == 1) n_rem ^= (prom[cnt >> 1] & 0x00FF);
    else              n_rem ^= (prom[cnt >> 1] >> 8);
    for (int n_bit = 8; n_bit > 0; n_bit--)
    {
      if (n_rem & 0x8000) n_rem = (n_rem << 1) ^ 0x3000;
      else                n_rem = n_rem << 1;
    }
  }
  prom[7] = saved;
  return (n_rem >> 12) & 0x000F;
}

} // namespace Espfc::Device::Baro
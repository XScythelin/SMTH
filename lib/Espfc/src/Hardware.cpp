#include "Device/Baro/BaroBMP085.hpp"
#include "Device/Baro/BaroBMP280.hpp"
#include "Device/Baro/BaroSPL06.hpp"
#include "Device/Baro/BaroMS5611.hpp"
#include "Device/BaroDevice.hpp"
#include "Device/Rangefinder/RangefinderVL53L0X.hpp"
#include "Device/RangefinderDevice.hpp"
#include "Device/GyroBMI160.h"
#include "Device/GyroDevice.h"
#include "Device/GyroICM20602.h"
#include "Device/GyroICM42688.h"
#include "Device/GyroLSM6DSO.h"
#include "Device/GyroMPU6050.h"
#include "Device/GyroMPU6500.h"
#include "Device/GyroMPU9250.h"
#include "Device/GyroICM20948.h"
#include "Device/Mag/MagAK8963.hpp"
#include "Device/Mag/MagHMC5883L.hpp"
#include "Device/Mag/MagQMC5883L.hpp"
#include "Device/Mag/MagQMC5883P.hpp"
#include "Hal/Gpio.h"
#include "Hardware.h"
#if defined(ESPFC_WIFI_ALT)
#include <ESP8266WiFi.h>
#elif defined(ESPFC_WIFI)
#include <WiFi.h>
#endif

namespace {
#if defined(ESPFC_SPI_0)
#if defined(ESP32C3) || defined(ESP32S3) || defined(ESP32S2)
static SPIClass SPI1(HSPI);
#elif defined(ESP32)
static SPIClass SPI1(VSPI);
#endif
static Espfc::Device::BusSPI spiBus(ESPFC_SPI_0_DEV);
#endif
#if defined(ESPFC_I2C_0)
static Espfc::Device::BusI2C i2cBus(WireInstance);
#endif
static Espfc::Device::BusSlave gyroSlaveBus;
static Espfc::Device::GyroMPU6050 mpu6050;
static Espfc::Device::GyroMPU6500 mpu6500;
static Espfc::Device::GyroMPU9250 mpu9250;
static Espfc::Device::GyroLSM6DSO lsm6dso;
static Espfc::Device::GyroICM20602 icm20602;
static Espfc::Device::GyroICM42688 icm42688;
static Espfc::Device::GyroBMI160 bmi160;
static Espfc::Device::GyroICM20948 icm20948;
static Espfc::Device::Mag::MagHMC5883L hmc5883l;
static Espfc::Device::Mag::MagQMC5883L qmc5883l;
static Espfc::Device::Mag::MagQMC5883P qmc5883p;
static Espfc::Device::Mag::MagAK8963 ak8963;
static Espfc::Device::Baro::BaroBMP085 bmp085;
static Espfc::Device::Baro::BaroBMP280 bmp280;
static Espfc::Device::Baro::BaroSPL06 spl06;
static Espfc::Device::Baro::BaroMS5611 ms5611;
static Espfc::Device::Rangefinder::RangefinderVL53L0X vl53l0x;

static bool hasModeCondition(const Espfc::Model& model, Espfc::FlightMode mode)
{
  for(size_t i = 0; i < Espfc::ACTUATOR_CONDITIONS; i++)
  {
    const auto& c = model.config.conditions[i];
    if(!(c.min < c.max)) continue;
    if(c.id == mode) return true;
  }
  return false;
}

static bool isSpiCs0SharedWithGpioOutput(const Espfc::Model& model)
{
  return hasModeCondition(model, Espfc::MODE_GPIO_OUTPUT) &&
         model.config.pin[Espfc::PIN_SPI_CS0] == 5;
}
} // namespace

namespace Espfc {

Hardware::Hardware(Model& model): _model(model) {}

int Hardware::begin()
{
  initBus();

  // Let sensors and bus lines settle after power-up/reset.
  delay(30);

  // Suppress I2C error callbacks during detection: every probe of a
  // non-existent address causes a NAK which fires onError. These are
  // expected during auto-detection and must not be counted as real errors
  // or allowed to corrupt the error counter that other subsystems watch.
#if defined(ESPFC_I2C_0)
  auto savedOnError = i2cBus.onError;
  i2cBus.onError = nullptr;
#endif

  // Do one normal detection pass and up to two short retry passes for
  // sensors that are still missing (helps with occasional startup races).
  detectGyro();
  detectMag();
  detectBaro();
  detectRangefinder();

  for (int retry = 0; retry < 2; retry++)
  {
    bool needGyro = _model.config.gyro.dev != GYRO_NONE && !_model.state.gyro.present;
    bool needBaro = _model.config.baro.dev != BARO_NONE && !_model.state.baro.present;
    bool needMag = _model.config.mag.dev != MAG_NONE && !_model.state.mag.present;
    bool needRangefinder = _model.isFeatureActive(FEATURE_RANGEFINDER) &&
                           _model.config.rangefinder.dev != RANGEFINDER_NONE &&
                           _model.config.rangefinder.dev != RANGEFINDER_MSP &&
                           !_model.state.rangefinder.present;

    if (!needGyro && !needBaro && !needMag && !needRangefinder) break;

    delay(25);

    if (needGyro) detectGyro();
    if (needMag) detectMag();
    if (needBaro) detectBaro();
    if (needRangefinder) detectRangefinder();
  }

#if defined(ESPFC_I2C_0)
  i2cBus.onError = savedOnError;
  _model.state.i2cErrorCount = 0; // clear spurious errors from detection
  _model.state.i2cErrorDelta = 0;
#endif

  return 1;
}

void Hardware::onI2CError()
{
  _model.state.i2cErrorCount++;
  _model.state.i2cErrorDelta++;
}

void Hardware::initBus()
{
#if defined(ESPFC_SPI_0)
  int spiResult = spiBus.begin(_model.config.pin[PIN_SPI_0_SCK], _model.config.pin[PIN_SPI_0_MOSI],
                               _model.config.pin[PIN_SPI_0_MISO]);
  _model.logger.info()
      .log(F("SPI"))
      .log(_model.config.pin[PIN_SPI_0_SCK])
      .log(_model.config.pin[PIN_SPI_0_MOSI])
      .log(_model.config.pin[PIN_SPI_0_MISO])
      .logln(spiResult);
#endif
#if defined(ESPFC_I2C_0)
  int i2cResult =
      i2cBus.begin(_model.config.pin[PIN_I2C_0_SDA], _model.config.pin[PIN_I2C_0_SCL], _model.config.i2cSpeed * 1000ul);
  i2cBus.onError = std::bind(&Hardware::onI2CError, this);
  _model.logger.info()
      .log(F("I2C"))
      .log(_model.config.pin[PIN_I2C_0_SDA])
      .log(_model.config.pin[PIN_I2C_0_SCL])
      .log(_model.config.i2cSpeed)
      .logln(i2cResult);
#endif
}

void Hardware::detectGyro()
{
  if (_model.config.gyro.dev == GYRO_NONE) return;

  const bool allowSpiBus = _model.config.gyro.bus == BUS_AUTO || _model.config.gyro.bus == BUS_SPI;
  const bool allowI2c = _model.config.gyro.bus == BUS_AUTO || _model.config.gyro.bus == BUS_I2C;
  const bool spiCs0ConflictsWithGpioOutput = isSpiCs0SharedWithGpioOutput(_model);
  const bool allowSpi = allowSpiBus && !spiCs0ConflictsWithGpioOutput;
  const int cs0 = _model.config.pin[PIN_SPI_CS0];
  const bool keepCs0LowAfterDetect = _model.config.pin[PIN_SPI_CS0] == 5;

  Device::GyroDevice* detectedGyro = nullptr;
#if defined(ESPFC_SPI_0)
  if (allowSpi && cs0 != -1)
  {
    Hal::Gpio::digitalWrite(cs0, HIGH);
    Hal::Gpio::pinMode(cs0, OUTPUT);

    auto releaseCs0 = [&]() {
      if (keepCs0LowAfterDetect)
      {
        Hal::Gpio::digitalWrite(cs0, LOW);
      }
    };

    if (!detectedGyro && detectDevice(mpu9250, spiBus, cs0)) detectedGyro = &mpu9250;
    releaseCs0();
    if (!detectedGyro && detectDevice(mpu6500, spiBus, cs0)) detectedGyro = &mpu6500;
    releaseCs0();
    if (!detectedGyro && detectDevice(icm20602, spiBus, cs0)) detectedGyro = &icm20602;
    releaseCs0();
    if (!detectedGyro && detectDevice(icm42688, spiBus, cs0)) detectedGyro = &icm42688;
    releaseCs0();
    if (!detectedGyro && detectDevice(bmi160, spiBus, cs0)) detectedGyro = &bmi160;
    releaseCs0();
    if (!detectedGyro && detectDevice(lsm6dso, spiBus, cs0)) detectedGyro = &lsm6dso;
    releaseCs0();

    if (detectedGyro && !spiCs0ConflictsWithGpioOutput) gyroSlaveBus.begin(&spiBus, detectedGyro->getAddress());
  }
#endif
#if defined(ESPFC_I2C_0)
  if (allowI2c && !detectedGyro && _model.config.pin[PIN_I2C_0_SDA] != -1 && _model.config.pin[PIN_I2C_0_SCL] != -1)
  {
    if (!detectedGyro && detectDevice(mpu9250, i2cBus)) detectedGyro = &mpu9250;
    if (!detectedGyro && detectDevice(mpu6500, i2cBus)) detectedGyro = &mpu6500;
    if (!detectedGyro && detectDevice(icm20602, i2cBus)) detectedGyro = &icm20602;
    if (!detectedGyro && detectDevice(icm20948, i2cBus)) detectedGyro = &icm20948;
    if (!detectedGyro && detectDevice(bmi160, i2cBus)) detectedGyro = &bmi160;
    if (!detectedGyro && detectDevice(mpu6050, i2cBus)) detectedGyro = &mpu6050;
    if (!detectedGyro && detectDevice(lsm6dso, i2cBus)) detectedGyro = &lsm6dso;
    if (detectedGyro) gyroSlaveBus.begin(&i2cBus, detectedGyro->getAddress());
  }
#endif

#if defined(ESPFC_SPI_0)
  if (keepCs0LowAfterDetect && cs0 != -1)
  {
    Hal::Gpio::digitalWrite(cs0, LOW);
    Hal::Gpio::pinMode(cs0, OUTPUT);
  }
#endif

  if (!detectedGyro) return;

  detectedGyro->setDLPFMode(_model.config.gyro.dlpf);
  _model.state.gyro.dev = detectedGyro;
  _model.state.gyro.present = (bool)detectedGyro;
  _model.state.accel.present = _model.state.gyro.present && _model.config.accel.dev != GYRO_NONE;
  _model.state.gyro.clock = detectedGyro->getRate();
}

void Hardware::detectMag()
{
  if (_model.config.mag.dev == MAG_NONE) return;

  Device::MagDevice* detectedMag = nullptr;
#if defined(ESPFC_I2C_0)
  if (_model.config.pin[PIN_I2C_0_SDA] != -1 && _model.config.pin[PIN_I2C_0_SCL] != -1)
  {
    if (!detectedMag && detectDevice(ak8963, i2cBus)) detectedMag = &ak8963;
    if (!detectedMag && detectDevice(hmc5883l, i2cBus)) detectedMag = &hmc5883l;
    if (!detectedMag && detectDevice(qmc5883l, i2cBus)) detectedMag = &qmc5883l;
    if (!detectedMag && detectDevice(qmc5883p, i2cBus)) detectedMag = &qmc5883p;
  }
#endif
  if (!isSpiCs0SharedWithGpioOutput(_model) && gyroSlaveBus.getBus())
  {
    if (!detectedMag && detectDevice(ak8963, gyroSlaveBus)) detectedMag = &ak8963;
    if (!detectedMag && detectDevice(hmc5883l, gyroSlaveBus)) detectedMag = &hmc5883l;
    if (!detectedMag && detectDevice(qmc5883l, gyroSlaveBus)) detectedMag = &qmc5883l;
    if (!detectedMag && detectDevice(qmc5883p, gyroSlaveBus)) detectedMag = &qmc5883p;
  }
  _model.state.mag.dev = detectedMag;
  _model.state.mag.present = (bool)detectedMag;
  _model.state.mag.rate = detectedMag ? detectedMag->getRate() : 0;
}

void Hardware::detectBaro()
{
  if (_model.config.baro.dev == BARO_NONE) return;

  const bool allowSpi = _model.config.baro.bus == BUS_AUTO || _model.config.baro.bus == BUS_SPI;
  const bool allowI2c = _model.config.baro.bus == BUS_AUTO || _model.config.baro.bus == BUS_I2C;
  const bool allowSlave = _model.config.baro.bus == BUS_AUTO || _model.config.baro.bus == BUS_SLV;

  Device::BaroDevice* detectedBaro = nullptr;
#if defined(ESPFC_SPI_0)
  if (allowSpi && _model.config.pin[PIN_SPI_CS1] != -1)
  {
    Hal::Gpio::digitalWrite(_model.config.pin[PIN_SPI_CS1], HIGH);
    Hal::Gpio::pinMode(_model.config.pin[PIN_SPI_CS1], OUTPUT);
    if (!detectedBaro && detectDevice(bmp280, spiBus, _model.config.pin[PIN_SPI_CS1])) detectedBaro = &bmp280;
    if (!detectedBaro && detectDevice(bmp085, spiBus, _model.config.pin[PIN_SPI_CS1])) detectedBaro = &bmp085;
    if (!detectedBaro && detectDevice(spl06, spiBus, _model.config.pin[PIN_SPI_CS1])) detectedBaro = &spl06;
    if (!detectedBaro && detectDevice(ms5611, spiBus, _model.config.pin[PIN_SPI_CS1])) detectedBaro = &ms5611; 
  }
#endif
#if defined(ESPFC_I2C_0)
  if (allowI2c && _model.config.pin[PIN_I2C_0_SDA] != -1 && _model.config.pin[PIN_I2C_0_SCL] != -1)
  {
    if (!detectedBaro && detectDevice(bmp280, i2cBus)) detectedBaro = &bmp280;
    if (!detectedBaro && detectDevice(bmp085, i2cBus)) detectedBaro = &bmp085;
    if (!detectedBaro && detectDevice(spl06, i2cBus)) detectedBaro = &spl06;
    if (!detectedBaro && detectDevice(ms5611, i2cBus)) detectedBaro = &ms5611;
  }
#endif
  if (allowSlave && !isSpiCs0SharedWithGpioOutput(_model) && gyroSlaveBus.getBus())
  {
    if (!detectedBaro && detectDevice(bmp280, gyroSlaveBus)) detectedBaro = &bmp280;
    if (!detectedBaro && detectDevice(bmp085, gyroSlaveBus)) detectedBaro = &bmp085;
    if (!detectedBaro && detectDevice(spl06, gyroSlaveBus)) detectedBaro = &spl06;
    // MS5611 uses command-only I2C transactions. Through gyro slave-bus this is
    // unreliable, so prefer direct I2C/SPI probing paths above.
  }

  _model.state.baro.dev = detectedBaro;
  _model.state.baro.present = (bool)detectedBaro;
}

void Hardware::detectRangefinder()
{
  if (!_model.isFeatureActive(FEATURE_RANGEFINDER)) return;
  if (_model.config.rangefinder.dev == RANGEFINDER_NONE) return;

  // external source fed over MSP/UART (e.g. MTF-02), no local bus probing
  if (_model.config.rangefinder.dev == RANGEFINDER_MSP)
  {
    _model.state.rangefinder.dev = nullptr;
    _model.state.rangefinder.present = false; // becomes true once MSP samples arrive
    return;
  }

  Device::RangefinderDevice* detectedRangefinder = nullptr;
#if defined(ESPFC_I2C_0)
  if (_model.config.pin[PIN_I2C_0_SDA] != -1 && _model.config.pin[PIN_I2C_0_SCL] != -1)
  {
    if (!detectedRangefinder && detectDevice(vl53l0x, i2cBus)) detectedRangefinder = &vl53l0x;
  }
#endif
  if (!isSpiCs0SharedWithGpioOutput(_model) && gyroSlaveBus.getBus())
  {
    if (!detectedRangefinder && detectDevice(vl53l0x, gyroSlaveBus)) detectedRangefinder = &vl53l0x;
  }

  _model.state.rangefinder.dev = detectedRangefinder;
  _model.state.rangefinder.present = (bool)detectedRangefinder;
}

void Hardware::restart(const Model& model)
{
  if (model.state.mixer.escMotor) model.state.mixer.escMotor->end();
  if (model.state.mixer.escServo) model.state.mixer.escServo->end();
#ifdef ESPFC_SERIAL_SOFT_0_WIFI
  WiFi.disconnect();
  WiFi.softAPdisconnect();
#endif
  delay(100);
  targetReset();
}

} // namespace Espfc
#ifndef _ESPFC_DEVICE_GYRO_ICM20948_H_
#define _ESPFC_DEVICE_GYRO_ICM20948_H_

#include "BusDevice.hpp"
#include "GyroMPU6050.h"
#include "helper_3dmath.h"
#include "Debug_Espfc.h"

namespace Espfc {

namespace Device {

// ====================== РЕГИСТРЫ ======================
#define ICM20948_RA_REG_BANK_SEL      0x7F

// Bank 0
#define ICM20948_RA_WHO_AM_I          0x00
#define ICM20948_WHO_AM_I             0xEA

#define ICM20948_RA_PWR_MGMT_1        0x06
#define ICM20948_RA_PWR_MGMT_2        0x07

// Bank 2
#define ICM20948_RA_GYRO_CONFIG_1     0x01  // [5:3] GYRO_DLPFCFG, [2:1] GYRO_FS_SEL, [0] GYRO_FCHOICE
#define ICM20948_RA_GYRO_SMPLRT_DIV   0x00
#define ICM20948_RA_ACCEL_CONFIG      0x14  // [5:3] ACCEL_DLPFCFG, [2:1] ACCEL_FS_SEL, [0] ACCEL_FCHOICE

// Данные (Bank 0)
#define ICM20948_RA_ACCEL_XOUT_H      0x2D
#define ICM20948_RA_GYRO_XOUT_H       0x33

/*
 * GYRO_CONFIG_1 (Bank 2, reg 0x01):
 *   bits [5:3] = GYRO_DLPFCFG — выбор частоты среза DLPF (0..7)
 *   bits [2:1] = GYRO_FS_SEL:
 *     00 = +-250  dps
 *     01 = +-500  dps
 *     10 = +-1000 dps
 *     11 = +-2000 dps
 *   bit  [0]   = GYRO_FCHOICE:
 *     0 = DLPF отключён (9kHz шум)
 *     1 = DLPF включён  <-- нам нужно это
 *
 * Значение 0x07 = 0b00000111 -> FS=11 (+-2000 dps), FCHOICE=1 (DLPF ON)
 *
 * ACCEL_CONFIG (Bank 2, reg 0x14):
 *   bits [5:3] = ACCEL_DLPFCFG — выбор частоты среза DLPF (0..7)
 *   bits [2:1] = ACCEL_FS_SEL:
 *     00 = +-2g
 *     01 = +-4g
 *     10 = +-8g
 *     11 = +-16g
 *   bit  [0]   = ACCEL_FCHOICE:
 *     0 = DLPF отключён
 *     1 = DLPF включён  <-- нам нужно это
 *
 * Значение 0x07 = 0b00000111 -> FS=11 (+-16g), FCHOICE=1 (DLPF ON)
 *
 * ВНИМАНИЕ: В ICM20948 FCHOICE=1 ВКЛЮЧАЕТ DLPF — логика обратна MPU6050!
 */

#define ICM20948_GYRO_CONFIG_2000DPS_DLPF_ON  0x07  // FS=11 (+-2000dps), FCHOICE=1 (DLPF ON)
#define ICM20948_ACCEL_CONFIG_16G_DLPF_ON     0x07  // FS=11 (+-16g),     FCHOICE=1 (DLPF ON)

class GyroICM20948 : public GyroMPU6050
{
public:
    GyroDeviceType getType() const override { return GYRO_ICM20948; }

    int begin(BusDevice *bus) override
    {
        if (tryInit(bus, 0x68)) return 1;
        if (tryInit(bus, 0x69)) return 1;
        D("icm20948: НЕ НАЙДЕН ни на 0x68, ни на 0x69");
        return 0;
    }

    int begin(BusDevice *bus, uint8_t addr) override
    {
        return tryInit(bus, addr);
    }

    bool testConnection() override
    {
        setBank(0);
        uint8_t whoami = 0;
        _bus->readByte(_addr, ICM20948_RA_WHO_AM_I, &whoami);
        D("icm20948 whoami @", _addr, "=", whoami, "(ожидаем 0xEA)");
        return whoami == ICM20948_WHO_AM_I;
    }

    void setDLPFMode(uint8_t mode) override
    {
        // ICM20948: биты [5:3] = GYRO_DLPFCFG / ACCEL_DLPFCFG
        // FCHOICE бит [0] = 1 включает DLPF (логика обратна MPU6050!)
        uint8_t dlpf = (mode > 7) ? 7 : mode;
        setBank(2);

        uint8_t gyroCfg = 0;
        _bus->readByte(_addr, ICM20948_RA_GYRO_CONFIG_1, &gyroCfg);
        gyroCfg = (gyroCfg & 0xC7) | (dlpf << 3);  // биты [5:3] = DLPFCFG, сохраняем [7:6] и [2:0]
        gyroCfg |= 0x01;                             // FCHOICE=1, DLPF ON
        _bus->writeByte(_addr, ICM20948_RA_GYRO_CONFIG_1, gyroCfg);

        uint8_t accelCfg = 0;
        _bus->readByte(_addr, ICM20948_RA_ACCEL_CONFIG, &accelCfg);
        accelCfg = (accelCfg & 0xC7) | (dlpf << 3); // биты [5:3] = DLPFCFG, сохраняем [7:6] и [2:0]
        accelCfg |= 0x01;                             // FCHOICE=1, DLPF ON
        _bus->writeByte(_addr, ICM20948_RA_ACCEL_CONFIG, accelCfg);

        setBank(0);
    }

    // ====================== ЧТЕНИЕ ДАННЫХ ======================
    // ВАЖНО: без setBank() — иначе relocation error на ESP32-S3
    int FAST_CODE_ATTR readGyro(VectorInt16& v) override
    {
        uint8_t buffer[6];
        if(!_bus->readFast(_addr, ICM20948_RA_GYRO_XOUT_H, 6, buffer))
            return 0;
        v.x = (((int16_t)buffer[0]) << 8) | buffer[1];
        v.y = (((int16_t)buffer[2]) << 8) | buffer[3];
        v.z = (((int16_t)buffer[4]) << 8) | buffer[5];
        return 1;
    }

    int readAccel(VectorInt16& v) override
    {
        uint8_t buffer[6];
        _bus->readFast(_addr, ICM20948_RA_ACCEL_XOUT_H, 6, buffer);

        v.x = (((int16_t)buffer[0]) << 8) | buffer[1];
        v.y = (((int16_t)buffer[2]) << 8) | buffer[3];
        v.z = (((int16_t)buffer[4]) << 8) | buffer[5];
        return 1;
    }

private:
    bool tryInit(BusDevice *bus, uint8_t addr)
    {
        _bus = bus;
        _addr = addr;
        D("icm20948: пробую адрес 0x", _addr);

        if (!testConnection()) return false;

        D("icm20948: сенсор найден по адресу 0x", _addr);
        return initSensor();
    }

    bool setBank(uint8_t bank)
    {
        bool ok = _bus->writeByte(_addr, ICM20948_RA_REG_BANK_SEL, bank << 4) == 1;
        delay(1);
        return ok;

    }

    int initSensor()
    {
        D("icm20948: начинаем инициализацию...");
        setBank(0);

        // Полный сброс устройства
        _bus->writeByte(_addr, ICM20948_RA_PWR_MGMT_1, 0x80);
        delay(100);

        // Выход из sleep, PLL от гироскопа как источник тактирования
        _bus->writeByte(_addr, ICM20948_RA_PWR_MGMT_1, 0x01);
        delay(25);

        // Включить гироскоп и акселерометр (все оси)
        _bus->writeByte(_addr, ICM20948_RA_PWR_MGMT_2, 0x00);

        setBank(2);

        // Гироскоп: +-2000 dps, DLPF включён (FCHOICE=1)
        // 0x07 = 0b00000111: FS_SEL=11 (+-2000dps), FCHOICE=1 (DLPF ON)
        _bus->writeByte(_addr, ICM20948_RA_GYRO_CONFIG_1, ICM20948_GYRO_CONFIG_2000DPS_DLPF_ON);

        // Акселерометр: +-16g, DLPF включён (FCHOICE=1)
        // 0x07 = 0b00000111: FS_SEL=11 (+-16g), FCHOICE=1 (DLPF ON)
        _bus->writeByte(_addr, ICM20948_RA_ACCEL_CONFIG, ICM20948_ACCEL_CONFIG_16G_DLPF_ON);

        setBank(0);
        D("icm20948: инициализация завершена успешно");
        return 1;
    }
};

} // namespace Device
} // namespace Espfc

#endif
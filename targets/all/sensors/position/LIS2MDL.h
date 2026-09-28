/*
 * sensors/position/LIS2MDL.h
 *
 * Driver for STMicroelectronics LIS2MDL 3-axis magnetometer
 */

#pragma once

#include <sensors/I2CSensor.h>

namespace sensors::position
{

class LIS2MDL : I2CSensor
{
public:
    //! LIS2MDL has a single fixed 7-bit I2C address
    static constexpr uint8_t DefaultAddress = 0x1E;

    LIS2MDL(bus::I2C i2c, uint8_t address = DefaultAddress)
        : I2CSensor(i2c, address)
    {
    }

    //! Field intensity per axis in gauss (NAN until first successful Measure)
    float GetFieldX() const { return x; }
    float GetFieldY() const { return y; }
    float GetFieldZ() const { return z; }

    //! Verifies the device and configures continuous mode
    async(Init);
    //! Reads a fresh field sample
    async(Measure);

protected:
    const char* DebugComponent() const { return "LIS2MDL"; }

private:
    enum struct Register : uint8_t
    {
        WhoAmI = 0x4F,
        CfgRegA = 0x60,
        CfgRegB = 0x61,
        CfgRegC = 0x62,
        Status = 0x67,
        OutXL = 0x68,   // X/Y/Z L/H auto-increment: 0x68..0x6D
    };

    static constexpr uint8_t WhoAmIValue = 0x40;

    // CFG_REG_A
    static constexpr uint8_t CfgA_ModeContinuous = 0x00;
    static constexpr uint8_t CfgA_Odr20Hz = 0x04;         // ODR = 01 << 2
    static constexpr uint8_t CfgA_CompTempEn = 0x80;
    // CFG_REG_C
    static constexpr uint8_t CfgC_Bdu = 0x10;             // block data update

    // 1.5 mgauss / LSB
    static constexpr float LsbToGauss = 1.5e-3f;

    bool init = false;
    float x = NAN, y = NAN, z = NAN;
};

}

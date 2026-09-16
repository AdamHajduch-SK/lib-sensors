/*
 * Copyright (c) 2026 triaxis s.r.o.
 * Licensed under the MIT license. See LICENSE.txt file in the repository root
 * for full license information.
 *
 * sensors/environment/TMP102.h
 *
 * Driver for Texas Instruments TMP102 digital temperature sensor
 */

#pragma once

#include <sensors/I2CSensor.h>

namespace sensors::environment
{

class TMP102 : I2CSensor
{
public:
    TMP102(bus::I2C i2c, uint8_t addr)
        : I2CSensor(i2c, addr)
    {
    }

    //! Probes the sensor - true if something acked at the configured address. TMP102 has no
    //! device-ID register to validate against (unlike e.g. MCP9600), so "did the bus transaction
    //! complete at all" is the only presence signal there is - same as the datasheet's own
    //! recommended way to detect it.
    async(Init);
    //! Reads the current temperature. No explicit configuration is written anywhere in this
    //! driver - continuous-conversion, 12-bit resolution is the power-on default, which is all a
    //! plain ambient reading needs.
    async(Measure);

    //! Gets the last measured temperature in degrees Celsius; NaN if not available
    float GetTemperature() const { return temperature; }

protected:
    const char* DebugComponent() const { return "TMP102"; }

private:
    enum struct Register : uint8_t
    {
        Temperature = 0,
        Config = 1,
    };

    float temperature = NAN;
};

}

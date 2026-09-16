/*
 * Copyright (c) 2026 triaxis s.r.o.
 * Licensed under the MIT license. See LICENSE.txt file in the repository root
 * for full license information.
 *
 * sensors/environment/TMP102.cpp
 */

#include "TMP102.h"

namespace sensors::environment
{

async(TMP102::Init)
async_def(
    uint16_t raw;
)
{
    MYDBG("Probing...");
    async_return(await(ReadRegister, Register::Temperature, f.raw, true));
}
async_end

async(TMP102::Measure)
async_def(
    uint16_t raw;
)
{
    if (!await(ReadRegister, Register::Temperature, f.raw))
    {
        async_return(false);
    }

    // 16-bit register, MSB first over the bus; POR default (continuous conversion, 12-bit
    // resolution) left-justifies the signed 12-bit result in the top 12 bits, so an arithmetic
    // >>4 on the correctly-endian-swapped 16-bit value both extracts and sign-extends it in one
    // step - 0.0625 degC/LSB.
    int16_t raw = (int16_t) FROM_BE16(f.raw) >> 4;
    temperature = raw * 0.0625f;
    MYTRACE("T=%.3q", f2q(temperature, 3));
    async_return(true);
}
async_end

}

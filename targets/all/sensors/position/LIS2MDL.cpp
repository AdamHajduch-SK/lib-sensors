/*
 * sensors/position/LIS2MDL.cpp
 */

#include "LIS2MDL.h"

namespace sensors::position
{

async(LIS2MDL::Init)
async_def(uint8_t v;)
{
    MYDBG("Reading WHO_AM_I...");
    if (!await(ReadRegister, Register::WhoAmI, f.v, true))
    {
        MYDBG("Not responding");
        async_return(false);
    }

    MYDBG("WHO_AM_I = %02X (expected %02X)", f.v, WhoAmIValue);
    if (f.v != WhoAmIValue)
    {
        async_return(false);
    }

    // continuous conversion, 20 Hz, on-chip temperature compensation
    f.v = CfgA_CompTempEn | CfgA_Odr20Hz | CfgA_ModeContinuous;
    if (!await(WriteRegister, Register::CfgRegA, f.v))
    {
        async_return(false);
    }

    // block data update so L/H bytes of a sample stay consistent while read
    f.v = CfgC_Bdu;
    if (!await(WriteRegister, Register::CfgRegC, f.v))
    {
        async_return(false);
    }

    MYDBG("Init complete");
    async_return(init = true);
}
async_end

async(LIS2MDL::Measure)
async_def(
    PACKED_UNALIGNED_STRUCT { int16_t x, y, z; } data;
)
{
    if (!init && !await(Init))
    {
        async_return(false);
    }

    // OUTX_L..OUTZ_H auto-increment; output is little-endian, signed
    if (!await(ReadRegister, Register::OutXL, f.data))
    {
        async_return(false);
    }

    x = int16_t(FROM_LE16(f.data.x)) * LsbToGauss;
    y = int16_t(FROM_LE16(f.data.y)) * LsbToGauss;
    z = int16_t(FROM_LE16(f.data.z)) * LsbToGauss;
    MYTRACE("X=%.3q Y=%.3q Z=%.3q", int(x * 1000), int(y * 1000), int(z * 1000));
    async_return(true);
}
async_end

}

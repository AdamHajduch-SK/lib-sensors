/*
 * Copyright (c) 2025 triaxis s.r.o.
 * Licensed under the MIT license. See LICENSE.txt file in the repository root
 * for full license information.
 *
 * sensors/environment/util.h
 */

#pragma once

#include <base/base.h>

namespace sensors::environment
{

//! Calculate altitude from pressure and QNH (pressure at sea level)
constexpr float PressureToAltitude(float pressure, float qnh)
{
    return 44330.0 * (1 - powf((pressure / qnh), 0.190295));
}

//! Calculate QNH (pressure at sea level) from pressure and known altitude - inverse of PressureToAltitude
constexpr float QnhFromAltitude(float pressure, float altitude)
{
    return pressure / powf(1 - altitude / 44330.0f, 1 / 0.190295f);
}

}

/*
 * Copyright (c) 2024 triaxis s.r.o.
 * Licensed under the MIT license. See LICENSE.txt file in the repository root
 * for full license information.
 *
 * sensors/gnss/MaxM10.h
 */

#pragma once

#include <kernel/kernel.h>

#include "NmeaGnssDevice.h"

namespace sensors::gnss
{

class MaxM10 : public NmeaGnssDevice
{
public:
    MaxM10(io::DuplexPipe pipe)
        : NmeaGnssDevice(pipe)
    {
    }

    async(SetBaudRate, unsigned baudRate) { return async_forward(SendMessageF, "PUBX,41,1,3,3,%u,0", baudRate); }

    //! Enables AssistNow Autonomous (CFG-ANA-USE_ANA) so the receiver predicts
    //! satellite orbits and can hot-start for up to ~3 days from BBR (V_BCKP).
    async(EnableAssistNow);

    //! Sets the navigation solution rate (CFG-RATE-MEAS), e.g. 1000 ms for 1 Hz or 100 ms for
    //! 10 Hz. Higher rates need a fast enough link - the module is driven at 921600 baud.
    async(SetMeasurementRate, unsigned periodMs);

    //! Queues a measurement rate change. It must not be sent from the caller's task: the transmit
    //! pipe takes a single writer, and a raw UBX frame slipped between the pieces of an outgoing
    //! NMEA sentence corrupts both, so the receiver answers neither ACK nor NAK.
    void RequestMeasurementRate(unsigned periodMs) { pendingRateMs = periodMs; requestPoll = true; }

    const UbxData& ExtendedData() const { return stableData; }

protected:
    virtual void OnMessage(io::Pipe::Iterator& message);
    virtual void OnIdle();

    async(PollRequest);

private:
    bool requestPoll = true;
    bool activePoll = false;
    //! Measurement period waiting to be sent, in ms; zero when there is nothing pending
    unsigned pendingRateMs = 0;
    UbxData data = { NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, 0, FixType::Unknown, -1 }, stableData = data;

    FixType ReadFixType(io::Pipe::Iterator& message);
};

}

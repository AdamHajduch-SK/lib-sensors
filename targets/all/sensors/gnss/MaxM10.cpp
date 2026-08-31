/*
 * Copyright (c) 2024 triaxis s.r.o.
 * Licensed under the MIT license. See LICENSE.txt file in the repository root
 * for full license information.
 *
 * sensors/gnss/MaxM10.cpp
 */

#include "MaxM10.h"

#define MYDBG(...)  DBGCL("MAX10", __VA_ARGS__)

namespace sensors::gnss
{


void MaxM10::OnMessage(io::Pipe::Iterator& message)
{
    if (requestPoll)
    {
        requestPoll = false;
        if (!activePoll)
        {
            activePoll = true;
            kernel::Task::Run(this, &MaxM10::PollRequest);
        }
        else
        {
            // a poll is already running - this request is silently dropped, relying on OnIdle
            // to set requestPoll again once it finishes. If PollRequest itself is stuck, this
            // fires forever with no new poll ever starting - that is exactly what we're hunting.
            MYDBG("requestPoll starved: a poll is already active (pendingRateMs=%u)", pendingRateMs);
        }
    }

    if (!message.Matches("PUBX,00,"))
    {
        return NmeaGnssDevice::OnMessage(message);
    }

    message.Skip(8);
    ReadDecimal(message);  // time
    ReadDecimal(message);  // latitude
    ReadChar(message);  // N/S
    ReadDecimal(message);   // longitude
    ReadChar(message);  // E/W
    data.altitude = ReadFloat(message);
    data.fixType = ReadFixType(message);
    data.hAcc = ReadFloat(message);
    data.vAcc = ReadFloat(message);
    data.groundSpeedKm = ReadFloat(message);
    data.course = ReadFloat(message);
    data.vVel = ReadFloat(message);
    data.diffAge = ReadNum(message, 10, -1);
    data.hdop = ReadFloat(message);
    data.vdop = ReadFloat(message);
    data.tdop = ReadFloat(message);
    data.numSat = ReadNum(message);
}

async(MaxM10::PollRequest)
async_def(
    unsigned ms;
)
{
    // Everything this driver sends goes out from here, one task. The transmit pipe tolerates a
    // single writer: SendMessageFV writes a sentence in three steps ('$', body, checksum), so a
    // raw UBX frame written from another task lands in the middle of it and destroys both. That
    // is why configuration frames went unanswered - the receiver never saw a valid message.
    MYDBG("PollRequest start (pendingRateMs=%u)", pendingRateMs);
    if (pendingRateMs)
    {
        f.ms = pendingRateMs;
        pendingRateMs = 0;
        MYDBG("PollRequest: awaiting SetMeasurementRate(%u)", f.ms);
        await(SetMeasurementRate, f.ms);
        MYDBG("PollRequest: SetMeasurementRate done");
    }

    MYDBG("PollRequest: awaiting SendMessage(PUBX,00)");
    await(SendMessage, "PUBX,00");
    MYDBG("PollRequest: SendMessage done");
    activePoll = false;
}
async_end

async(MaxM10::EnableAssistNow)
async_def(
    uint8_t msg[17];
)
{
    // UBX-CFG-VALSET enabling CFG-ANA-USE_ANA (0x10230001 = 1) in the RAM layer.
    // Applied on every boot; the predicted-orbit (AOP) data itself is kept in the
    // battery-backed BBR (V_BCKP), extending hot-start capability across power cycles.
    // If the key/frame is rejected the receiver replies UBX-ACK-NAK and nothing else
    // changes (no cold start), so this is safe even if unsupported.
    static const uint8_t body[] = {
        0x06, 0x8A,             // class, id: CFG-VALSET
        0x09, 0x00,             // payload length = 9 (little-endian)
        0x00,                   // version
        0x01,                   // layers = RAM
        0x00, 0x00,             // reserved
        0x01, 0x00, 0x23, 0x10, // key CFG-ANA-USE_ANA (0x10230001, little-endian)
        0x01,                   // value = 1 (enable)
    };

    f.msg[0] = 0xB5;
    f.msg[1] = 0x62;
    uint8_t ckA = 0, ckB = 0;
    for (unsigned i = 0; i < sizeof(body); i++)
    {
        f.msg[2 + i] = body[i];
        ckA += body[i];
        ckB += ckA;
    }
    f.msg[2 + sizeof(body)] = ckA;
    f.msg[3 + sizeof(body)] = ckB;

    await(SendRaw, Span(f.msg, sizeof(f.msg)));
}
async_end

async(MaxM10::SetMeasurementRate, unsigned periodMs)
async_def(
    uint8_t msg[18];
)
{
    // UBX-CFG-VALSET writing CFG-RATE-MEAS (0x30210001, U2, milliseconds) to the RAM layer.
    // 1000 = 1 Hz (default), 100 = 10 Hz for speed flying. Rejected keys are answered with
    // UBX-ACK-NAK and change nothing, so an unsupported receiver simply keeps its rate.
    uint8_t body[] = {
        0x06, 0x8A,             // class, id: CFG-VALSET
        0x0A, 0x00,             // payload length = 10 (little-endian)
        0x00,                   // version
        0x01,                   // layers = RAM
        0x00, 0x00,             // reserved
        0x01, 0x00, 0x21, 0x30, // key CFG-RATE-MEAS (0x30210001, little-endian)
        uint8_t(periodMs), uint8_t(periodMs >> 8),  // value (U2, little-endian)
    };

    f.msg[0] = 0xB5;
    f.msg[1] = 0x62;
    uint8_t ckA = 0, ckB = 0;
    for (unsigned i = 0; i < sizeof(body); i++)
    {
        f.msg[2 + i] = body[i];
        ckA += body[i];
        ckB += ckA;
    }
    f.msg[2 + sizeof(body)] = ckA;
    f.msg[3 + sizeof(body)] = ckB;

    await(SendRaw, Span(f.msg, sizeof(f.msg)));
}
async_end

void MaxM10::OnIdle()
{
    NmeaGnssDevice::OnIdle();
    stableData = data;
    requestPoll = true;
}

FixType MaxM10::ReadFixType(io::Pipe::Iterator& message)
{
    char id[2];
    message.Read(id);
    message.Consume(',');

    switch (ID(id))
    {
        case ID("NF"): return FixType::None;
        case ID("DR"): return FixType::DeadReckoning;
        case ID("G2"): return FixType::Std2D;
        case ID("G3"): return FixType::Std3D;
        case ID("D2"): return FixType::Diff2D;
        case ID("D3"): return FixType::Diff3D;
        case ID("RK"): return FixType::Combined;
        case ID("TT"): return FixType::TimeOnly;
        default: return FixType::Unknown;
    }
}

}

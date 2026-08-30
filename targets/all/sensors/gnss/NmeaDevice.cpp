/*
 * Copyright (c) 2024 triaxis s.r.o.
 * Licensed under the MIT license. See LICENSE.txt file in the repository root
 * for full license information.
 *
 * sensors/gnss/NmeaDevice.cpp
 */

#include "NmeaDevice.h"

#include <kernel/kernel.h>

#define MYDBG(...)      DBGCL("NMEA", __VA_ARGS__)

//#define NMEA_TRACE    1

#if NMEA_TRACE
#define MYTRACE(...)    MYDBG(__VA_ARGS__)
#else
#define MYTRACE(...)
#endif

namespace sensors::gnss
{

async(NmeaDevice::Init)
async_def_sync()
{
    kernel::Task::Run(this, &NmeaDevice::Receiver);
}
async_end

async(NmeaDevice::Receiver)
async_def(size_t len)
{
    MYDBG("Starting receiver");
    for (;;)
    {
        // skip the last message
        rx.Advance(f.len);

        // Both delimiter waits below MUST be bounded: if the stream carries garbage
        // without the expected delimiter (e.g. baud-rate mismatch after a warm MCU
        // reset, when the GNSS module is still at the high baud rate), an unbounded
        // wait deadlocks with a full RX pipe - the parser waits for a delimiter that
        // never comes while the pipe cannot accept more data, and the low-level
        // receiver busy-spins, freezing the entire system. On timeout, discard
        // everything buffered and resync.

        // skip to next '$', detect idle
        auto res = await_catch(rx.RequireUntil, '$', Timeout::Milliseconds(10));
        if (res.Success())
        {
            f.len = res.Value();
        }
        else
        {
            OnIdle();
            res = await_catch(rx.RequireUntil, '$', Timeout::Seconds(1));
            if (!res.Success())
            {
                MYDBG("no '$' in stream, discarding %u bytes", rx.Available());
                f.len = 0;
                rx.Advance(rx.Available());
                continue;
            }
            f.len = res.Value();
        }
#if TRACE
        // Anything before the '$' is not NMEA and gets skipped silently, which is where the
        // receiver's binary answers to our configuration went - we sent UBX frames for years
        // without ever seeing whether they were accepted. Report an ACK/NAK so a rejected
        // setting stops looking like a firmware bug.
        if (f.len >= 10)
        {
            uint8_t head[4] = {};
            for (unsigned i = 0; i < 4; i++) { head[i] = (uint8_t)rx.Peek((size_t)i); }
            if (head[0] == 0xB5 && head[1] == 0x62 && head[2] == 0x05)
            {
                MYDBG("UBX %s from receiver", head[3] ? "ACK" : "NAK");
            }
        }
#endif

        rx.Advance(f.len);
        // wait until the entire message is buffered (NMEA sentences are < 100 bytes,
        // so 500 ms is generous even at the lowest baud rate)
        res = await_catch(rx.RequireUntil, '\n', Timeout::Milliseconds(500));
        if (!res.Success())
        {
            MYDBG("no '\\n' after '$', discarding %u bytes", rx.Available());
            f.len = 0;
            rx.Advance(rx.Available());
            continue;
        }
        f.len = res.Value();

        uint8_t csum = 0;
        auto iter = rx.Enumerate(f.len);
        while (iter && *iter != '*')
        {
            csum ^= *iter;
            ++iter;
        }

        if (!iter)
        {
            MYDBG("Invalid message - '*' not found");
            continue;
        }

        if (iter.Available() != 5)
        {
            MYDBG("Invalid message - encountered '*' %d chars too early", iter.Available() - 5);
            continue;
        }

        char chsumh = *++iter, chsuml = *++iter;
        int csumh = parse_nibble(chsumh), csuml = parse_nibble(chsuml);
        if (csumh < 0 || csuml < 0)
        {
            MYDBG("Invalid checksum character %c or %c", chsumh, chsuml);
            continue;
        }

        if ((csumh << 4 | csuml) != csum)
        {
            MYDBG("Checksum error - expected %02X, received %02X", csum, (csumh << 4 | csuml));
            continue;
        }
        ++iter;
        if (!iter.Matches("\r\n"))
        {
            MYDBG("Invalid message - not terminated with CRLF");
            continue;
        }

#if TRACE && NMEA_TRACE
        DBGC("NMEA", "<< ");
        for (auto s: rx.EnumerateSpans(f.len - 5))
        {
            _DBG("%b", s);
        }
        _DBGCHAR('\n');
#endif

        iter = rx.Enumerate(f.len - 5);
        OnMessage(iter);
    }
}
async_end

async(NmeaDevice::SendMessageFV, Timeout timeout, const char* format, va_list va)
async_def(
    Timeout timeout;
    io::PipePosition start;
    uint8_t checksum;
)
{
    f.timeout = timeout.MakeAbsolute();

    await(tx.Write, "$", f.timeout);
    f.start = tx.Position();
    await(tx.WriteFV, timeout, format, va);

#if NMEA_TRACE
    DBGC("NMEA", ">> ");
#endif
    for (char c : tx.EnumerateFrom(f.start))
    {
        f.checksum ^= c;
#if NMEA_TRACE
        _DBGCHAR(c);
#endif
    }
#if NMEA_TRACE
    _DBGCHAR('\n');
#endif

    await(tx.WriteFTimeout, timeout, "*%02X\r\n", f.checksum);
}
async_end

int NmeaDevice::ReadNum(io::Pipe::Iterator& message, unsigned base, int errorValue)
{
    auto dec = ReadDecimal(message, base);
    if (dec.divisor == 0) { return errorValue; }
    if (dec.divisor != 1)
    {
        MYDBG("error parsing integer - unexpected decimal point");
        return errorValue;
    }
    return dec.value;
}

Packed<Decimal> NmeaDevice::ReadDecimalImpl(io::Pipe::Iterator& message, unsigned base)
{
    bool negative = message.Consume('-');

    int n = 0;
    int div = 0;
    bool error = true;
    while (char c = message.Read(0))
    {
        if (c == ',') { break; }

        error = false;

        if (c == '.')
        {
            if (div == 0) { div = 1; }
            else
            {
                MYDBG("error parsing decimal - encountered multiple decimal points");
                error = true;
            }
            continue;
        }

        int nib = parse_nibble(c, base);
        if (nib == -1)
        {
            MYDBG("error parsing number - encountered %c", *message);
            error = true;
        }
        else
        {
            n = n * base + nib;
            if (div)
            {
                div *= 10;
            }
        }
    }

    return pack<Decimal>({ negative ? -n : n, error ? 0 : div ? div : 1 });
}

char NmeaDevice::ReadChar(io::Pipe::Iterator& message)
{
    char c = message.Read();
    if (c != ',')
    {
        message.Consume(',');
    }
    return c;
}

float NmeaDevice::ReadDeg(io::Pipe::Iterator& message)
{
    auto dec = ReadDecimal(message);
    if (!dec.divisor) { return NAN; }
    int degrees = dec.value / (dec.divisor * 100);
    int minutes = dec.value % (dec.divisor * 100);
    return degrees + (minutes / (60.0f * dec.divisor));
}

float NmeaDevice::ReadFloat(io::Pipe::Iterator& message)
{
    auto dec = ReadDecimal(message);
    if (!dec.divisor) { return NAN; }
    return (float)dec.value / dec.divisor;
}

}

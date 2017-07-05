/************************************************************************
 * Copyright 2018 © Kévin Lesénéchal <kevin.lesenechal@gmail.com>       *
 * This file is part of MotoLogger.                                     *
 *                                                                      *
 * MotoLogger is free software: you can redistribute it and/or modify   *
 * it under the terms of the GNU General Public License as published by *
 * the Free Software Foundation, either version 3 of the License, or    *
 * (at your option) any later version.                                  *
 *                                                                      *
 * MotoLogger is distributed in the hope that it will be useful,        *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of       *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the        *
 * GNU General Public License for more details.                         *
 *                                                                      *
 * You should have received a copy of the GNU General Public License    *
 * along with MotoLogger.  If not, see <http://www.gnu.org/licenses/>.  *
 ************************************************************************/

/* Tested on:
 *   - Yamaha MT-07 2014
 *
 * Should work on:
 *   - Yamaha MT-125
 *   - Yamaha MT-03
 *   - Yamaha MT-07
 *   - Yamaha MT-09
 *   - Yamaha MT-10
 *
 * CAN bus bitrate: 500 kb/s
 */

#include <motologger/sample.hpp>
#include <motologger/can_frame.hpp>

#include <stdint.h>

#if defined(YAMAHA_MT07_2014) || defined(YAMAHA_MT07_2014_35KW)

#define THROTTLE_MIN_VALUE 0x22
#ifdef YAMAHA_MT07_2014_35KW
  // 35 kW version: 95 possible values (0x22 - 0x80)
  #define THROTTLE_MAX_VALUE 0x80
#else
  // Full version: 173 possible values (0x22 - 0xce)
  #define THROTTLE_MAX_VALUE 0xce
#endif
#define THROTTLE_U16_SCALE (65536 / (THROTTLE_MIN_VALUE - THROTTLE_MAX_VALUE))

using motologger::can_frame;
using motologger::sample;

namespace motologger {
namespace {

bool extract_gear(sample& sample, uint8_t val)
{
    switch (val)
    {
    case 0x00: sample.gear = 0;   break; // Neutral ('-' on dashboard)
    case 0x20: sample.gear = 1;   break;
    case 0x40: sample.gear = 2;   break;
    case 0x60: sample.gear = 3;   break;
    case 0x80: sample.gear = 4;   break;
    case 0xa0: sample.gear = 5;   break;
    case 0xc0: sample.gear = 6;   break;
    case 0xe0: sample.gear = 255; break; // Blank display on dashboard
    default: return false;
    }
    return true;
}

bool extract_throttle(sample& sample, uint8_t val8)
{
    if (val8 < THROTTLE_MIN_VALUE || val8 > THROTTLE_MAX_VALUE) {
        return false;
    }

    uint16_t val16 = static_cast<uint16_t>(
        (val8 - THROTTLE_MIN_VALUE) * THROTTLE_U16_SCALE
    );

    if (val8 == THROTTLE_MAX_VALUE) {
        sample.throttle = 255;
    } else {
        sample.throttle = static_cast<uint8_t>(val16 / 256);
    }

    return true;
}

bool extract_engine_speed(sample& sample, const uint8_t* data)
{
    sample.engine_speed = static_cast<uint16_t>(data[2] * 100);

    return true;
}

bool extract_engine_temp(sample& sample, uint8_t data)
{
    if (data < 0x70 || data > 0xea) {
        return false;
    }

    int val = 40 + (data - 0x70) * 625 / 1000;
    sample.engine_temp = static_cast<uint8_t>(val);

    return true;
}

bool extract_air_temp(sample& sample, uint8_t data)
{
    if (data < 0x21 || data > 0xcf) {
        return false;
    }

    int val = -9 + (data - 0x21) * 625 / 1000;
    sample.air_temp = static_cast<int8_t>(val);

    return true;
}

bool extract_speed(sample& sample, uint8_t data)
{
    sample.indicated_speed = static_cast<uint16_t>(data * 10);

    return true;
}

}

#define DLC_CHECK(n) do { if (frame.size != (n)) return false; } while (0)

/**
 * @brief Extract information from a CAN frame into a recording sample
 * @param frame  The CAN frame to extract information from
 * @param sample A sample instance to write information to
 * @return false if the frame ID is unknown or some values fall outside their
 *         known boundaries for a given frame ID, otherwise true
 * @note Returning false does not prevent this function from writing values into
 *       @p sample for the known values encountered
 */
bool decode_can_frame(const can_frame& frame, sample& sample)
{
    bool nok;

    switch (frame.id)
    {
    case 0x020a: // Engine speed (1 (2?) byte(s)) + speed + ???
        DLC_CHECK(8);
        nok = !extract_engine_speed(sample, frame.data);
        nok |= !extract_speed(sample, frame.data[4]);
        return !nok;

    case 0x0216: // Throttle + ???
        DLC_CHECK(7);
        extract_throttle(sample, frame.data[0]);
        break;

    case 0x022e: // ???
        DLC_CHECK(2);
        return frame.data[0] == 0x00 && frame.data[1] == 0x00;

    case 0x0236: // Gear
        DLC_CHECK(1);
        return extract_gear(sample, frame.data[0]);

    case 0x023a: // Diagnostic
        DLC_CHECK(3);
        /*
         * data[0] seems to be a bit field:
         *   1: ? (always 0)
         *   2: ? (always 1)
         *   3: starter?
         *   4: ? (always 0)
         *   5: ? (always 0)
         *   6: ? (always 0)
         *   7: ? (always 0)
         *   8: ? (always 0)
         */
        sample.flags.engine_alert = frame.data[0] & 0x0c;
        return (frame.data[0] & 0xf0) == 0
               && frame.data[1] == 0xff
               && frame.data[2] == 0xff;

    case 0x023e: // Engine temp (1 byte) + air temp (1 byte) + ???
        DLC_CHECK(4);
        nok = !extract_engine_temp(sample, frame.data[0]);
        nok |= !extract_air_temp(sample, frame.data[1]);
        nok |= frame.data[2] != 0x00;
        nok |= frame.data[3] != 0x00;
        return !nok;

    default:
        return false;
    }

    return true;
}

} // ns motologger

#endif /* YAMAHA_MT07_2014 || YAMAHA_MT07_2014_35KW */

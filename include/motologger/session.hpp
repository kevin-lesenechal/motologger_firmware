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

/**
 * @file   session.hpp
 * @author Kévin Lesénéchal <kevin.lesenechal@gmail.com>
 * @date   2017-07-20
 */

#pragma once

#include <ff.h>

namespace motologger {

struct sample;

enum device_capability : uint64_t
{
    engine_speed    = 1LL << 0,
    indicated_speed = 1LL << 1,
    true_speed      = 1LL << 2,
    gear            = 1LL << 3,
    throttle        = 1LL << 4,
    clutch          = 1LL << 5,
    front_brake     = 1LL << 6,
    rear_brake      = 1LL << 7,
    flags_int       = 0xffff00,
    lean_angle      = 1LL << 24,
    pitch_angle     = 1LL << 25,
    engine_temp     = 1LL << 26,
    air_temp        = 1LL << 27,
    x_accel         = 1LL << 28,
    y_accel         = 1LL << 29,
    z_accel         = 1LL << 30,
    fuel_consump    = 1LL << 31,
    fuel_quantity   = 1LL << 32,
    odometer        = 1LL << 33
};

class session final
{
public:
    session(time_t        start_time_s,
            uint16_t      start_time_ms) noexcept;
    session(const session&) = delete;
    session& operator=(const session&) = delete;
    ~session();

    bool open(const char* file_name);
    bool write_header();
    bool write_sample(const sample* sample);
    void sync();

public:
    static constexpr int    FORMAT_VERSION = 1;
    static constexpr size_t HEADER_SIZE = 32;
    static constexpr int    FLUSH_SAMPLES_COUNT = 50;

private:
    FIL      _fh;
    time_t   _start_time_s;
    uint16_t _start_time_ms;
};

} // ns motologger

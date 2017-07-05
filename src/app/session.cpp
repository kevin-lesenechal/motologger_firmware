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
 * @file   session.cpp
 * @author Kévin Lesénéchal <kevin.lesenechal@gmail.com>
 * @date   2017-07-20
 */

#include "motologger/session.hpp"
#include "motologger/sample.hpp"

#include <string.h>
#include <utils.hpp>
#include <main.hpp>
#include <motologger/application.hpp>

namespace motologger {

session::session(time_t        start_time_s,
                 uint16_t      start_time_ms) noexcept
  : _start_time_s(start_time_s),
    _start_time_ms(start_time_ms)
{}

session::~session()
{
    f_close(&_fh);
}

bool session::open(const char* file_name)
{
    FRESULT  res;

    if ((res = f_open(&_fh, file_name, FA_WRITE | FA_CREATE_NEW)) != FR_OK)
    {
        panic("Failed to open '%s' file: %d", file_name, res);
        return false;
    }
    return true;
}

bool session::write_header()
{
    char     buff[HEADER_SIZE];
    uint64_t capabilities;
    uint16_t time_resolution;
    uint32_t start_time_s;
    uint16_t start_time_ms;
    UINT     size_written;

    memset(buff, 0, sizeof buff);
    memcpy(buff, "MOTOLOG", 8);
    buff[8] = FORMAT_VERSION;
    capabilities = to_be64(conf::CAPABILITIES);
    memcpy(buff + 9,  &capabilities, sizeof capabilities);
    start_time_s = to_be32(static_cast<uint32_t>(_start_time_s));
    memcpy(buff + 17, &start_time_s, sizeof start_time_s);
    start_time_ms = to_be16(_start_time_ms);
    memcpy(buff + 21, &start_time_ms, sizeof start_time_ms);
    time_resolution = to_be16(conf::TIME_RESOLUTION);
    memcpy(buff + 23, &time_resolution, sizeof time_resolution);
    buff[25] = conf::SAMPLE_SIZE;
    return f_write(&_fh, buff, sizeof buff, &size_written) == FR_OK;
}

#define WRITE_DATA_32(_field_)                  \
    do { if (conf::CAPABILITIES & (_field_)) \
    {                                           \
        u32 = to_be32(sample->_field_);         \
        memcpy(buff + i, &u32, sizeof u32);     \
        i += 4;                                 \
    } } while (false)

#define WRITE_DATA_16(_field_)                  \
    do { if (conf::CAPABILITIES & (_field_)) \
    {                                           \
        u16 = to_be16(sample->_field_);         \
        memcpy(buff + i, &u16, sizeof u16);     \
        i += 2;                                 \
    } } while (false)

#define WRITE_DATA_8(_field_)                   \
    do { if (conf::CAPABILITIES & (_field_)) \
        buff[i++] = sample->_field_;            \
    } while (false)

bool session::write_sample(const sample* sample)
{
    ssize_t  i = 0;
    char     buff[sample::MAX_SIZE];
    uint16_t u16;
    uint32_t u32;
    UINT     size_written;

    memset(buff, 0, sizeof buff);
    WRITE_DATA_16(engine_speed);
    WRITE_DATA_16(indicated_speed);
    WRITE_DATA_16(true_speed);
    WRITE_DATA_8 (gear);
    WRITE_DATA_8 (throttle);
    WRITE_DATA_8 (clutch);
    WRITE_DATA_8 (front_brake);
    WRITE_DATA_8 (rear_brake);
    WRITE_DATA_16(flags_int);
    WRITE_DATA_8 (lean_angle);
    WRITE_DATA_8 (pitch_angle);
    WRITE_DATA_8 (engine_temp);
    WRITE_DATA_8 (air_temp);
    WRITE_DATA_16(x_accel);
    WRITE_DATA_16(y_accel);
    WRITE_DATA_16(z_accel);
    WRITE_DATA_8 (fuel_consump);
    WRITE_DATA_8 (fuel_quantity);
    WRITE_DATA_32(odometer);
    write_sample_extra(buff, sample);
    return f_write(&_fh, buff, conf::SAMPLE_SIZE, &size_written) == FR_OK;
}

void session::sync()
{
    f_sync(&_fh);
}

} // ns motologger

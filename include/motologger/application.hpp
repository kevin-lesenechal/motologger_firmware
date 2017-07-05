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
 * @file   application.hpp
 * @author Kévin Lesénéchal <kevin.lesenechal@gmail.com>
 * @date   2017-07-20
 */

#pragma once

#include "sample.hpp"
#include "session.hpp"
#include "moto.hpp"

#include <ff.h>
#include <array>

namespace motologger {

struct can_frame;

namespace conf {

constexpr uint8_t  SAMPLE_SIZE      = 12;
constexpr uint16_t TIME_RESOLUTION  = 100;
constexpr uint64_t CAPABILITIES     = moto::CAPABILITIES
                                      | x_accel | y_accel | z_accel;

} // ns conf

class application final
{
public:
    application() noexcept;
    ~application();

    void run();
    void halt();
    void write_sample();
    void on_can_frame(const can_frame* frame);

private:
    bool open_sess_file();
    void log_can_frame(const can_frame* frame);
    void read_imu();

private:
    bool    _run;
    sample  _sample;
    uint32_t _sample_i;
    FIL     _unknown_file;
    session _session;
    int     _indicated_speed_sum;
    int     _indicated_speed_n;
};

extern application* app;

} // ns motologger

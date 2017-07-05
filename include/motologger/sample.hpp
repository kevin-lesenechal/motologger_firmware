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
 * @file   sample.hpp
 * @author Kévin Lesénéchal <kevin.lesenechal@gmail.com>
 * @date   2017-07-20
 */

#pragma once

#include <cstdint>

namespace motologger {

enum sample_flag
{
    engine_on     = 1 << 0,
    left_turn     = 1 << 1,
    right_turn    = 1 << 2,
    warnings      = 1 << 3,
    horn          = 1 << 4,
    high_beam     = 1 << 5,
    temp_alert    = 1 << 6,
    engine_alert  = 1 << 7,
    oil_alert     = 1 << 8,
    battery_alert = 1 << 9,
    abs_alert     = 1 << 10,
    front_abs     = 1 << 11,
    rear_abs      = 1 << 12,
    kickstand     = 1 << 13
};

struct sample_flags
{
    bool engine_on    : 1;
    bool left_turn    : 1;
    bool right_turn   : 1;
    bool warnings     : 1;
    bool horn         : 1;
    bool highbeam     : 1;
    bool temp_alert   : 1;
    bool engine_alert : 1;
    bool oil_alert    : 1;
    bool battery_alert: 1;
    bool abs_alert    : 1;
    bool front_abs    : 1;
    bool rear_abs     : 1;
    bool kickstand    : 1;
};

struct sample
{
    uint16_t engine_speed;
    uint16_t indicated_speed;
    uint16_t true_speed;
    uint8_t  gear;
    uint8_t  throttle;
    uint8_t  clutch;
    uint8_t  front_brake;
    uint8_t  rear_brake;
    union
    {
        sample_flags flags;
        uint16_t    flags_int;
    };
    int8_t   lean_angle;
    int8_t   pitch_angle;
    uint8_t  engine_temp;
    int8_t   air_temp;
    int16_t  x_accel;
    int16_t  y_accel;
    int16_t  z_accel;
    uint8_t  fuel_consump;
    uint8_t  fuel_quantity;
    uint32_t odometer;

    static constexpr std::size_t MAX_SIZE = 128;
};

} // ns motologger

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
 * @file   can_frame.hpp
 * @author Kévin Lesénéchal <kevin.lesenechal@gmail.com>
 * @date   2017-07-20
 */

#pragma once

#include <cstdint>

namespace motologger {

struct can_frame
{
    uint32_t id;
    uint8_t  size;
    uint8_t  data[8];
};

} // ns motologger

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

#include "utils.hpp"

uint16_t to_be16(uint16_t n)
{
  uint8_t* s = (uint8_t*)&n;
  return (uint16_t)(s[0] << 8 | s[1]);
}

uint32_t to_be32(uint32_t n)
{
  uint8_t* s = (uint8_t*)&n;
  return (uint32_t)(s[0] << 24 | s[1] << 16 | s[2] << 8 | s[3]);
}

uint64_t to_be64(uint64_t n)
{
  uint8_t* s = (uint8_t*)&n;
  return (uint64_t)(
    (uint64_t)s[0] << 56 | (uint64_t)s[1] << 48 | (uint64_t)s[2] << 40
    | (uint64_t)s[3] << 32 | s[4] << 24 | s[5] << 16 | s[6] << 8  | s[7]
  );
}

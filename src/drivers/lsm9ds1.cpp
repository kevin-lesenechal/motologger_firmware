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
 * @file   lsm9ds1.cpp
 * @author Kévin Lesénéchal <kevin.lesenechal@gmail.com>
 * @date   2017-07-21
 */

#include "drivers/lsm9ds1.hpp"

#include <main.hpp>
#include <hw.hpp>

namespace drivers {

void lsm9ds1::init()
{
    uint8_t accel_id = read(WHO_AM_I_XG), magneto_id = read(WHO_AM_I_M);
    if (accel_id != ACCEL_GYRO_ID || magneto_id != MAGNETO_ID) {
        panic("lsm9ds1: Invalid IDs, got XG = %#hhu and M = %#hhu",
              accel_id, magneto_id);
    }
    uint8_t reg = 0;

    //---- Accelerometer
    write(CTRL_REG5_XL, 0x38); // Enable X, Y, Z
    reg = 0;
    reg |= static_cast<uint8_t>(accel_freq) << 5;
    reg |= static_cast<uint8_t>(accel_scale) << 3;
    write(CTRL_REG6_XL, reg);
    write(CTRL_REG7_XL, 0x00);

    //---- Gyroscope
    reg = 0;
    reg |= static_cast<uint8_t>(gyro_freq) << 5;
    reg |= static_cast<uint8_t>(gyro_scale) << 3;
    write(CTRL_REG1_G, reg);
    write(CTRL_REG2_G, 0x00);
    write(CTRL_REG3_G, 0x00);
    write(CTRL_REG4, 0x38);
    write(ORIENT_CFG_G, 0x00);

    //---- Magnetometer
    reg = 0;
    reg |= 1 << 7 | 0x03 << 5;
    reg |= static_cast<uint8_t>(mag_freq) << 2;
    write(CTRL_REG1_M, reg);
    reg = 0;
    reg |= static_cast<uint8_t>(mag_scale) << 5;
    write(CTRL_REG2_M, reg);
    write(CTRL_REG3_M, 0x00);
    write(CTRL_REG4_M, 0x00);
    write(CTRL_REG5_M, 0x00);
}

lsm9ds1::vector<int16_t> lsm9ds1::get_accel_i()
{
    int16_t values[3];

    read(OUT_X_L_XL, reinterpret_cast<uint8_t*>(values), sizeof values);
    return vector<int16_t>{values[0], values[1], values[2]};
}

lsm9ds1::vector<double> lsm9ds1::get_accel_f()
{
    double scale;

    switch (accel_scale)
    {
    case accel_scale::scale_2g:  scale = 4.0  / 65536.0; break;
    case accel_scale::scale_4g:  scale = 8.0  / 65536.0; break;
    case accel_scale::scale_8g:  scale = 16.0 / 65536.0; break;
    case accel_scale::scale_16g: scale = 32.0 / 65536.0; break;
    }

    auto v = get_accel_i();
    return vector<double>{v.x * scale, v.y * scale, v.z * scale};
}

lsm9ds1::vector<int16_t> lsm9ds1::get_gyro_i()
{
    int16_t values[3];

    read(OUT_X_L_G, reinterpret_cast<uint8_t*>(values), sizeof values);
    return vector<int16_t>{values[0], values[1], values[2]};
}

lsm9ds1::vector<double> lsm9ds1::get_gyro_f()
{
    double scale;

    switch (gyro_scale)
    {
    case gyro_scale::scale_245dps:  scale = 490.0  / 65536.0; break;
    case gyro_scale::scale_500dps:  scale = 1000.0 / 65536.0; break;
    case gyro_scale::scale_2000dps: scale = 4000.0 / 65536.0; break;
    }

    auto v = get_gyro_i();
    return vector<double>{v.x * scale, v.y * scale, v.z * scale};
}

lsm9ds1::vector<int16_t> lsm9ds1::get_mag_i()
{
    int16_t values[3];

    read(OUT_X_L_M, reinterpret_cast<uint8_t*>(values), sizeof values);
    return vector<int16_t>{values[0], values[1], values[2]};
}

lsm9ds1::vector<double> lsm9ds1::get_mag_f()
{
    double scale;

    switch (mag_scale)
    {
    case mag_scale::scale_4g:  scale = 4.0  / 65536.0; break;
    case mag_scale::scale_8g:  scale = 8.0  / 65536.0; break;
    case mag_scale::scale_12g: scale = 12.0 / 65536.0; break;
    case mag_scale::scale_16g: scale = 16.0 / 65536.0; break;
    }

    auto v = get_mag_i();
    return vector<double>{v.x * scale, v.y * scale, v.z * scale};
}

void lsm9ds1::read(accel_gyro_register reg, uint8_t* data, size_t size)
{
    hw::io::imu_write(ACCEL_GYRO_ADDR,
            reinterpret_cast<uint8_t*>(&reg), sizeof reg);
    hw::io::imu_read(ACCEL_GYRO_ADDR, data, size);
}

uint8_t lsm9ds1::read(lsm9ds1::accel_gyro_register reg)
{
    uint8_t r;
    read(reg, &r, sizeof r);
    return r;
}

void lsm9ds1::read(magneto_register reg, uint8_t* data, size_t size)
{
    hw::io::imu_write(MAGNETO_ADDR,
            reinterpret_cast<uint8_t*>(&reg), sizeof reg);
    hw::io::imu_read(MAGNETO_ADDR, data, size);
}

uint8_t lsm9ds1::read(lsm9ds1::magneto_register reg)
{
    uint8_t r;
    read(reg, &r, sizeof r);
    return r;
}

void lsm9ds1::write(lsm9ds1::accel_gyro_register reg, uint8_t value)
{
    uint8_t buffer[2] = {reg, value};
    hw::io::imu_write(ACCEL_GYRO_ADDR, buffer, sizeof buffer);
}

void lsm9ds1::write(lsm9ds1::magneto_register reg, uint8_t value)
{
    uint8_t buffer[2] = {reg, value};
    hw::io::imu_write(MAGNETO_ADDR, buffer, sizeof buffer);
}

} // ns drivers

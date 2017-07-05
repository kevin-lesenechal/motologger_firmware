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
 * @file   lsm9ds1.hpp
 * @author Kévin Lesénéchal <kevin.lesenechal@gmail.com>
 * @date   2017-07-21
 */

#pragma once

#include <cstdint>
#include <cstdlib>

namespace drivers {

class lsm9ds1
{
public:
    template<typename T>
    struct vector
    {
        T x;
        T y;
        T z;
    };

    void init();

    vector<int16_t> get_accel_i();
    vector<int16_t> get_gyro_i();
    vector<int16_t> get_mag_i();
    vector<double>  get_accel_f();
    vector<double>  get_gyro_f();
    vector<double>  get_mag_f();

public:
    enum class accel_scale
    {
        scale_2g  = 0x00,
        scale_4g  = 0x02,
        scale_8g  = 0x03,
        scale_16g = 0x01
    };

    enum class accel_freq
    {
        off        = 0x00,
        freq_10hz  = 0x01,
        freq_50hz  = 0x02,
        freq_119hz = 0x03,
        freq_238hz = 0x04,
        freq_476hz = 0x05,
        freq_952hz = 0x06
    };

    enum class gyro_scale
    {
        scale_245dps  = 0x00,
        scale_500dps  = 0x01,
        scale_2000dps = 0x03
    };

    enum class gyro_freq
    {
        off        = 0x00,
        freq_15hz  = 0x01,
        freq_60hz  = 0x02,
        freq_119hz = 0x03,
        freq_238hz = 0x04,
        freq_476hz = 0x05,
        freq_95hz  = 0x06
    };

    enum class mag_scale
    {
        scale_4g  = 0x00,
        scale_8g  = 0x01,
        scale_12g = 0x02,
        scale_16g = 0x03
    };

    enum class mag_freq
    {
        freq_0_625hz = 0x00,
        freq_1_25hz  = 0x01,
        freq_2_5hz   = 0x02,
        freq_5hz     = 0x03,
        freq_10hz    = 0x04,
        freq_20hz    = 0x05,
        freq_40hz    = 0x06,
        freq_80hz    = 0x07
    };

    accel_scale accel_scale = accel_scale::scale_2g;
    accel_freq  accel_freq  = accel_freq::off;
    gyro_scale  gyro_scale  = gyro_scale::scale_245dps;
    gyro_freq   gyro_freq   = gyro_freq::off;
    mag_scale   mag_scale   = mag_scale::scale_4g;
    mag_freq    mag_freq    = mag_freq::freq_0_625hz;

private:
    static constexpr uint8_t ACCEL_GYRO_ADDR = 0x6b;
    static constexpr uint8_t MAGNETO_ADDR    = 0x1e;
    static constexpr uint8_t ACCEL_GYRO_ID   = 0x68;
    static constexpr uint8_t MAGNETO_ID      = 0x3d;

    enum accel_gyro_register : uint8_t
    {
      ACT_THS          = 0x04, ACT_DUR          = 0x05, INT_GEN_CFG_XL   = 0x06,
      INT_GEN_THS_X_XL = 0x07, INT_GEN_THS_Y_XL = 0x08, INT_GEN_THS_Z_XL = 0x09,
      INT_GEN_DUR_XL   = 0x0a, REFERENCE_G      = 0x0b, INT1_CTRL        = 0x0c,
      INT2_CTRL        = 0x0d, WHO_AM_I_XG      = 0x0f, CTRL_REG1_G      = 0x10,
      CTRL_REG2_G      = 0x11, CTRL_REG3_G      = 0x12, ORIENT_CFG_G     = 0x13,
      INT_GEN_SRC_G    = 0x14, OUT_TEMP_L       = 0x15, OUT_TEMP_H       = 0x16,
      STATUS_REG_0     = 0x17, OUT_X_L_G        = 0x18, OUT_X_H_G        = 0x19,
      OUT_Y_L_G        = 0x1a, OUT_Y_H_G        = 0x1b, OUT_Z_L_G        = 0x1c,
      OUT_Z_H_G        = 0x1d, CTRL_REG4        = 0x1e, CTRL_REG5_XL     = 0x1f,
      CTRL_REG6_XL     = 0x20, CTRL_REG7_XL     = 0x21, CTRL_REG8        = 0x22,
      CTRL_REG9        = 0x23, CTRL_REG10       = 0x24, INT_GEN_SRC_XL   = 0x26,
      STATUS_REG_1     = 0x27, OUT_X_L_XL       = 0x28, OUT_X_H_XL       = 0x29,
      OUT_Y_L_XL       = 0x2a, OUT_Y_H_XL       = 0x2b, OUT_Z_L_XL       = 0x2c,
      OUT_Z_H_XL       = 0x2d, FIFO_CTRL        = 0x2e, FIFO_SRC         = 0x2f,
      INT_GEN_CFG_G    = 0x30, INT_GEN_THS_XH_G = 0x31, INT_GEN_THS_XL_G = 0x32,
      INT_GEN_THS_YH_G = 0x33, INT_GEN_THS_YL_G = 0x34, INT_GEN_THS_ZH_G = 0x35,
      INT_GEN_THS_ZL_G = 0x36, INT_GEN_DUR_G    = 0x37
    };

    enum magneto_register : uint8_t
    {
      OFFSET_X_REG_L_M = 0x05, OFFSET_X_REG_H_M = 0x06, OFFSET_Y_REG_L_M = 0x07,
      OFFSET_Y_REG_H_M = 0x08, OFFSET_Z_REG_L_M = 0x09, OFFSET_Z_REG_H_M = 0x0a,
      WHO_AM_I_M       = 0x0f, CTRL_REG1_M      = 0x20, CTRL_REG2_M      = 0x21,
      CTRL_REG3_M      = 0x22, CTRL_REG4_M      = 0x23, CTRL_REG5_M      = 0x24,
      STATUS_REG_M     = 0x27, OUT_X_L_M        = 0x28, OUT_X_H_M        = 0x29,
      OUT_Y_L_M        = 0x2a, OUT_Y_H_M        = 0x2b, OUT_Z_L_M        = 0x2c,
      OUT_Z_H_M        = 0x2d, INT_CFG_M        = 0x30, INT_SRC_M        = 0x31,
      INT_THS_L_M      = 0x32, INT_THS_H_M      = 0x33
    };

private:
    void    read(accel_gyro_register reg, uint8_t* data, size_t size);
    uint8_t read(accel_gyro_register reg);
    void    read(magneto_register reg, uint8_t* data, size_t size);
    uint8_t read(magneto_register reg);
    void    write(accel_gyro_register reg, uint8_t value);
    void    write(magneto_register reg, uint8_t value);
};

} // ns drivers

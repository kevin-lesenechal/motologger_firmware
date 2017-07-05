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
 * @file   application.cpp
 * @author Kévin Lesénéchal <kevin.lesenechal@gmail.com>
 * @date   2017-07-20
 */

#include "motologger/application.hpp"
#include "motologger/can_frame.hpp"
#include "main.hpp"

#include <hw.hpp>

#include <string.h>
#include <utils.hpp>
#include <ff.h>
#include <tgmath.h>

namespace motologger {

application* app = nullptr;

application::application() noexcept
    : _run(true),
      _sample_i(0),
      _session(hw::get_rtc_timestamp(), 0),
      _indicated_speed_sum(0),
      _indicated_speed_n(0)
{
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET);
    puts("\n\nHello MotoLogger!");

    open_sess_file();
    _session.write_header();
    if (FRESULT res = f_open(&_unknown_file, "unknown_frames",
                             FA_WRITE | FA_OPEN_ALWAYS);
        res != FR_OK) {
        panic("Failed to open 'unknown_frames' file: %d", res);
    }

    app = this;

    hw::start_sampling_timer();
    hw::start_can();
}

application::~application()
{
    hw::stop_can();
    hw::stop_sampling_timer();
    f_close(&_unknown_file);
}

void application::run()
{
    printf("Reading CAN frames...\n");

    while (_run) {
        hw::sleep();
    }
}

bool application::open_sess_file()
{
    char     filename[32];
    uint16_t last_id = 0;
    DIR      dir;
    FILINFO  finfo;

    if (FRESULT res = f_opendir(&dir, "/"); res != FR_OK) {
        panic("Failed to open root directory: %d", res);
        return false;
    }
    while (f_readdir(&dir, &finfo) == FR_OK) {
        uint16_t scanned_id;
        if (finfo.fname[0] == '\0') break;
        if (sscanf(finfo.fname, "session_%04hu.motolog", &scanned_id) == 1) {
            if (scanned_id > last_id)
                last_id = scanned_id;
        }
    }
    (void) f_closedir(&dir);
    snprintf(filename, sizeof filename, "session_%04hu.motolog", last_id + 1);
    _session.open(filename);

    return true;
}

void application::write_sample()
{
    static uint8_t count = 0;

    read_imu();
    hw::set_irq_enabled(false);
    _sample.indicated_speed = static_cast<uint16_t>(roundf(
        static_cast<float>(_indicated_speed_sum) / _indicated_speed_n
    ));
    _indicated_speed_sum = 0;
    _indicated_speed_n = 0;
    _session.write_sample(&_sample);
    ++_sample_i;
    hw::set_irq_enabled(true);
    if (count++ >= session::FLUSH_SAMPLES_COUNT) {
        _session.sync();
        f_sync(&_unknown_file);
        count = 0;
    }
}

void application::on_can_frame(const can_frame* frame)
{
    bool known;

    hw::set_irq_enabled(false);
    known = decode_can_frame(*frame, _sample);
    _indicated_speed_sum += _sample.indicated_speed;
    ++_indicated_speed_n;
    hw::set_irq_enabled(true);

    if (!known) {
        log_can_frame(frame);
    }
}

#ifdef CONFIG_HAS_IMU
void application::read_imu()
{
    using hw::dev::imu;

    auto v = imu.get_accel_i();

    hw::set_irq_enabled(false);
    _sample.x_accel = v.x;
    _sample.y_accel = v.y;
    _sample.z_accel = v.z;
    hw::set_irq_enabled(true);
}
#else
void application::read_imu() {}
#endif

// TODO: add timestamp (+ ms)
void application::log_can_frame(const can_frame* frame)
{
    char     buffer[17]; // Sample number (4) + ID (4) + DLC (1) + payload (8)
    size_t   size_written;
    uint32_t id, sample_i;

    printf("[%.8lx] ", frame->id);
    for (uint8_t i = 0; i < frame->size; ++i) {
        printf("%.2x ", frame->data[i]);
    }
    puts("");

    id = to_be32(frame->id);
    sample_i = to_be32(_sample_i);
    memset(buffer, 0, sizeof buffer);
    memcpy(buffer, &sample_i, sizeof sample_i);
    memcpy(buffer + 4, &id, sizeof id);
    buffer[8] = frame->size;
    memcpy(buffer + 9, frame->data, frame->size);
    (void) f_write(&_unknown_file, buffer, 9 + frame->size, &size_written);
}

void application::halt()
{
    printf("Shutting down...\n");
    _run = false;
}

} // ns motologger

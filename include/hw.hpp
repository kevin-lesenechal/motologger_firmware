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

#pragma once

#include <stm32f4xx.h>
#include <drivers/lsm9ds1.hpp>
#include <time.h>

#define CONFIG_HAS_UART
//#define CONFIG_HAS_IMU

namespace hw {

enum class led {
    active,
    charging,
    error
};

void    init();
void    sleep();
void    stop();
void    set_irq_enabled(bool enabled);
void    set_led(led l, bool enabled);
void    start_sampling_timer();
void    stop_sampling_timer();
void    start_can();
void    stop_can();
time_t  get_rtc_timestamp();
void    set_rtc_timestamp(time_t time);

} // ns hw

namespace hw::dev {

extern CAN_HandleTypeDef    can;
extern I2C_HandleTypeDef    i2c;
extern SD_HandleTypeDef     sd;
extern TIM_HandleTypeDef    timer;
extern UART_HandleTypeDef   uart;
extern RTC_HandleTypeDef    rtc;
extern drivers::lsm9ds1     imu;

} // ns hw::dev

namespace hw::conf {

constexpr int LSE_FREQ      = 32'768; // Hz
constexpr int HSI_FREQ      = 16'000'000; // Hz

constexpr int SYSCLK        = 36'000'000; // Hz
constexpr int HCLK          = 36'000'000; // Hz
constexpr int PCLK1_FREQ    = 18'000'000; // Hz
constexpr int PCLK2_FREQ    = 36'000'000; // Hz

constexpr int RCC_PLLM      = 8;
constexpr int RCC_PLLN      = 72;
constexpr int RCC_PLLP      = 4;
constexpr int RCC_PLLQ      = 3;
constexpr int RCC_PLLR      = 2;

constexpr int RCC_AHB_PRESCALER     = 1;
constexpr int RCC_APB1_PRESCALER    = 2;
constexpr int RCC_APB2_PRESCALER    = 1;

static_assert(LSE_FREQ == LSE_VALUE);
static_assert(HSI_FREQ == HSI_VALUE);
static_assert(SYSCLK == HSI_FREQ / RCC_PLLM * RCC_PLLN / RCC_PLLP);
static_assert(HCLK == SYSCLK / RCC_AHB_PRESCALER);
static_assert(PCLK1_FREQ == HCLK / RCC_APB1_PRESCALER);
static_assert(PCLK2_FREQ == HCLK / RCC_APB2_PRESCALER);

} // ns hw::conf

namespace hw::io {

void imu_read(uint8_t addr, uint8_t* data, size_t size);
void imu_write(uint16_t addr, uint8_t* data, size_t size);

} // ns hw::io

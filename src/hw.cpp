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

#include "hw.hpp"

#include <main.hpp>
#include <hw.hpp>

namespace hw::dev {

CAN_HandleTypeDef   can;
I2C_HandleTypeDef   i2c;
SD_HandleTypeDef    sd;
TIM_HandleTypeDef   timer;
UART_HandleTypeDef  uart;
RTC_HandleTypeDef   rtc;
drivers::lsm9ds1    imu;

}

static bool rtc_initialized = false;

void hw::sleep()
{
    HAL_SuspendTick();
    HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
    HAL_ResumeTick();
}

void hw::stop()
{
    set_led(led::active, false);
    set_led(led::error, false);
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
}

void hw::set_irq_enabled(bool enabled)
{
    if (enabled) {
        __enable_irq();
    } else {
        __disable_irq();
    }
}

void hw::set_led(hw::led l, bool enabled)
{
    uint16_t pin;

    switch (l) {
    case led::active:   pin = GPIO_PIN_0; break;
    case led::charging: pin = GPIO_PIN_1; break;
    case led::error:    pin = GPIO_PIN_2; break;
    }

    HAL_GPIO_WritePin(GPIOE, pin, enabled ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void hw::start_sampling_timer()
{
    using hw::dev::timer;

    HAL_TIM_Base_Start_IT(&timer);
}

void hw::stop_sampling_timer()
{
    using hw::dev::timer;

    HAL_TIM_Base_Stop_IT(&timer);
}

void hw::start_can()
{
    using hw::dev::can;

    if (int err = HAL_CAN_Start(&can); err != HAL_OK) {
        panic("can: Couldn't start CAN module: %d", err);
    }

    if (int err = HAL_CAN_ActivateNotification(&can, CAN_IT_RX_FIFO0_MSG_PENDING);
        err != HAL_OK) {
        panic("can: Couldn't enable interrupts: %d", err);
    }
}

void hw::stop_can()
{
    using hw::dev::can;

    if (int err = HAL_CAN_Stop(&can); err != HAL_OK) {
        panic("can: Couldn't stop CAN module: %d", err);
    }
}

time_t hw::get_rtc_timestamp()
{
    using hw::dev::rtc;

    RTC_DateTypeDef rtc_date;
    RTC_TimeTypeDef rtc_time;

    if (!rtc_initialized) {
        return 0;
    }

    HAL_RTC_GetTime(&rtc, &rtc_time, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&rtc, &rtc_date, RTC_FORMAT_BIN);

    tm t;
    t.tm_year = rtc_date.Year + 100;
    t.tm_mday = rtc_date.Date;
    t.tm_mon  = rtc_date.Month - 1;
    t.tm_hour = rtc_time.Hours;
    t.tm_min  = rtc_time.Minutes;
    t.tm_sec  = rtc_time.Seconds;

    return mktime(&t);
}

void hw::set_rtc_timestamp(time_t time)
{
    using hw::dev::rtc;

    tm t;
    RTC_TimeTypeDef rtc_time;
    RTC_DateTypeDef rtc_date;

    gmtime_r(&time, &t);

    rtc_date.Year       = static_cast<uint8_t>(t.tm_year - 100);
    rtc_date.Month      = static_cast<uint8_t>(t.tm_mon + 1);
    rtc_date.Date       = static_cast<uint8_t>(t.tm_mday);
    rtc_time.Hours      = static_cast<uint8_t>(t.tm_hour);
    rtc_time.Minutes    = static_cast<uint8_t>(t.tm_min);
    rtc_time.Seconds    = static_cast<uint8_t>(t.tm_sec);

    HAL_RTC_SetTime(&rtc, &rtc_time, RTC_FORMAT_BIN);
    HAL_RTC_SetDate(&rtc, &rtc_date, RTC_FORMAT_BIN);

    rtc_initialized = true;
}

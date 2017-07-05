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
#include "main.hpp"

#include <motologger/can_frame.hpp>
#include <motologger/application.hpp>

extern "C" {

void SysTick_Handler()
{
    HAL_IncTick();
    HAL_SYSTICK_IRQHandler();
}

void CAN1_RX0_IRQHandler()
{
    HAL_CAN_IRQHandler(&hw::dev::can);
}

void TIM6_IRQHandler()
{
    HAL_TIM_IRQHandler(&hw::dev::timer);
}

void EXTI0_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}

//----------------------------------------------------------------------------//

void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
    if (pin == GPIO_PIN_0 && motologger::app != nullptr) {
        motologger::app->halt();
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef*)
{
    if (motologger::app != nullptr) {
        motologger::app->write_sample();
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* can)
{
    CAN_RxHeaderTypeDef header;
    motologger::can_frame frame;

    if (HAL_CAN_GetRxMessage(can, CAN_RX_FIFO0,
                             &header, frame.data) != HAL_OK) {
        panic("can: Couldn't read RX message");
    }

    frame.id   = header.StdId;
    frame.size = static_cast<uint8_t>(header.DLC);

    if (motologger::app != nullptr) {
        motologger::app->on_can_frame(&frame);
    }
}

} // extern "C"

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

#include <hw.hpp>
#include <main.hpp>
#include <motologger/application.hpp>

namespace {

/**
 * @brief Initialises HSI internal RC oscillator and PLL
 *
 * HSI RC runs at 16 MHz and is used solely through a PLL to provide the SYSCLK
 * at 36 MHz.
 */
void init_rcc_oscillators()
{
    RCC_OscInitTypeDef osc;

    osc.OscillatorType  = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSE;
    osc.HSIState        = RCC_HSI_ON;
    osc.HSICalibrationValue = hw::conf::HSI_FREQ / 1'000'000; // 16 MHz
    osc.LSEState        = RCC_LSE_ON;
    osc.PLL.PLLState    = RCC_PLL_ON;
    osc.PLL.PLLSource   = RCC_PLLSOURCE_HSI;
    osc.PLL.PLLM        = hw::conf::RCC_PLLM; // 16 / 8 = 2 MHz
    osc.PLL.PLLN        = hw::conf::RCC_PLLN; // 2 * 72 = 144 MHz
    osc.PLL.PLLP        = hw::conf::RCC_PLLP; // 144 / 4 = 36 MHz -> SYSCLK
    osc.PLL.PLLQ        = hw::conf::RCC_PLLQ; // 144 / 3 = 48 MHz (unused)
    osc.PLL.PLLR        = hw::conf::RCC_PLLR; // Unused

    if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
        panic("rcc: failed to initialize oscillators");
    }
}

void init_rcc_clocks()
{
    RCC_ClkInitTypeDef clock;

    static_assert(hw::conf::RCC_AHB_PRESCALER == 1);
    static_assert(hw::conf::RCC_APB1_PRESCALER == 2);
    static_assert(hw::conf::RCC_APB2_PRESCALER == 1);

    clock.ClockType =
        RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
        | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clock.SYSCLKSource      = RCC_SYSCLKSOURCE_PLLCLK; // 36 MHz
    clock.AHBCLKDivider     = RCC_SYSCLK_DIV1;  // 36 / 1 = 36 MHz -> AHB (HCLK)
    clock.APB1CLKDivider    = RCC_HCLK_DIV2;    // 36 / 2 = 18 MHz -> APB1
    clock.APB2CLKDivider    = RCC_HCLK_DIV1;    // 36 / 1 = 36 MHz -> APB2

    if (HAL_RCC_ClockConfig(&clock, FLASH_LATENCY_1) != HAL_OK) {
        panic("rcc: failed to initialize clocks");
    }
}

void init_rcc_periph_clock()
{
    RCC_PeriphCLKInitTypeDef clock;

    clock.PeriphClockSelection  = RCC_PERIPHCLK_SDIO | RCC_PERIPHCLK_CLK48
                                  | RCC_PERIPHCLK_RTC;
    clock.Clk48ClockSelection   = RCC_CLK48CLKSOURCE_PLLQ;
    clock.SdioClockSelection    = RCC_SDIOCLKSOURCE_CLK48;
    clock.RTCClockSelection     = RCC_RTCCLKSOURCE_LSE; // 32.768 kHz
    if (HAL_RCCEx_PeriphCLKConfig(&clock) != HAL_OK) {
        panic("rcc: failed to initialize peripheral clock");
    }
}

void init_rcc()
{
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    init_rcc_oscillators();
    init_rcc_clocks();
    init_rcc_periph_clock();

    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

void init_nvic()
{
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

    HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
    HAL_NVIC_SetPriority(BusFault_IRQn,         0, 0);
    HAL_NVIC_SetPriority(UsageFault_IRQn,       0, 0);
    HAL_NVIC_SetPriority(SVCall_IRQn,           0, 0);
    HAL_NVIC_SetPriority(DebugMonitor_IRQn,     0, 0);
    HAL_NVIC_SetPriority(PendSV_IRQn,           0, 0);
    HAL_NVIC_SetPriority(SysTick_IRQn,          0, 0);
    HAL_NVIC_SetPriority(EXTI0_IRQn,            0, 0);
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn,         1, 0);

    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

void init_gpio()
{
    GPIO_InitTypeDef gpio;

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_5|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_8, GPIO_PIN_RESET);

    // PE2 PE3 PE0 PE1
    gpio.Pin    = GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_0 | GPIO_PIN_1;
    gpio.Mode   = GPIO_MODE_OUTPUT_PP;
    gpio.Pull   = GPIO_NOPULL;
    gpio.Speed  = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOE, &gpio);

    // PE4
    gpio.Pin    = GPIO_PIN_4;
    gpio.Mode   = GPIO_MODE_AF_PP;
    gpio.Pull   = GPIO_NOPULL;
    gpio.Speed  = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF8_DFSDM1;
    HAL_GPIO_Init(GPIOE, &gpio);

    // PF0
    gpio.Pin    = GPIO_PIN_0;
    gpio.Mode   = GPIO_MODE_AF_PP;
    gpio.Pull   = GPIO_NOPULL;
    gpio.Speed  = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF12_FSMC;
    HAL_GPIO_Init(GPIOF, &gpio);

    // PF5 PF11 PF12
    gpio.Pin    = GPIO_PIN_5 | GPIO_PIN_11 | GPIO_PIN_12;
    gpio.Mode   = GPIO_MODE_OUTPUT_PP;
    gpio.Pull   = GPIO_NOPULL;
    gpio.Speed  = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOF, &gpio);

    // PF6 PF7
    gpio.Pin    = GPIO_PIN_6 | GPIO_PIN_7;
    gpio.Mode   = GPIO_MODE_AF_PP;
    gpio.Pull   = GPIO_NOPULL;
    gpio.Speed  = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF9_QSPI;
    HAL_GPIO_Init(GPIOF, &gpio);

    // PF8 PF9
    gpio.Pin    = GPIO_PIN_8 | GPIO_PIN_9;
    gpio.Mode   = GPIO_MODE_AF_PP;
    gpio.Pull   = GPIO_NOPULL;
    gpio.Speed  = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF10_OTG_FS;
    HAL_GPIO_Init(GPIOF, &gpio);

    // PC2
    gpio.Pin    = GPIO_PIN_2;
    gpio.Mode   = GPIO_MODE_AF_PP;
    gpio.Pull   = GPIO_NOPULL;
    gpio.Speed  = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF8_DFSDM1;
    HAL_GPIO_Init(GPIOC, &gpio);

    // PA4
    gpio.Pin    = GPIO_PIN_4;
    gpio.Mode   = GPIO_MODE_AF_PP;
    gpio.Pull   = GPIO_NOPULL;
    gpio.Speed  = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF6_SPI3;
    HAL_GPIO_Init(GPIOA, &gpio);

    // PB1
    gpio.Pin    = GPIO_PIN_1;
    gpio.Mode   = GPIO_MODE_AF_PP;
    gpio.Pull   = GPIO_NOPULL;
    gpio.Speed  = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF8_DFSDM1;
    HAL_GPIO_Init(GPIOB, &gpio);

    // PB2
    gpio.Pin    = GPIO_PIN_2;
    gpio.Mode   = GPIO_MODE_AF_PP;
    gpio.Pull   = GPIO_NOPULL;
    gpio.Speed  = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF9_QSPI;
    HAL_GPIO_Init(GPIOB, &gpio);

    // PE7 PE8 PE9 PE10 PE11 PE12 PE13 PE14 PE15
    gpio.Pin    = GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10
                  | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14
                  | GPIO_PIN_15;
    gpio.Mode   = GPIO_MODE_AF_PP;
    gpio.Pull   = GPIO_NOPULL;
    gpio.Speed  = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF12_FSMC;
    HAL_GPIO_Init(GPIOE, &gpio);

    // PB10
    gpio.Pin    = GPIO_PIN_10;
    gpio.Mode   = GPIO_MODE_AF_OD;
    gpio.Pull   = GPIO_PULLUP;
    gpio.Speed  = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF4_I2C2;
    HAL_GPIO_Init(GPIOB, &gpio);

    // PB11
    gpio.Pin    = GPIO_PIN_11;
    gpio.Mode   = GPIO_MODE_AF_PP;
    gpio.Pull   = GPIO_NOPULL;
    gpio.Speed  = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOB, &gpio);

    // PB12 PB4
    gpio.Pin    = GPIO_PIN_12|GPIO_PIN_4;
    gpio.Mode   = GPIO_MODE_AF_PP;
    gpio.Pull   = GPIO_NOPULL;
    gpio.Speed  = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF5_SPI3;
    HAL_GPIO_Init(GPIOB, &gpio);

    // PD8 PD9 PD10 PD14 PD15 PD4 PD5 PD7
    gpio.Pin    = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_14
                  | GPIO_PIN_15 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_7;
    gpio.Mode   = GPIO_MODE_AF_PP;
    gpio.Pull   = GPIO_NOPULL;
    gpio.Speed  = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF12_FSMC;
    HAL_GPIO_Init(GPIOD, &gpio);

    // PD11
    gpio.Pin    = GPIO_PIN_11;
    gpio.Mode   = GPIO_MODE_OUTPUT_PP;
    gpio.Pull   = GPIO_NOPULL;
    gpio.Speed  = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &gpio);

    // PG2 PG5
    gpio.Pin    = GPIO_PIN_2 | GPIO_PIN_5;
    gpio.Mode   = GPIO_MODE_IT_RISING;
    gpio.Pull   = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOG, &gpio);

    // PG4 PG7
    gpio.Pin    = GPIO_PIN_4 | GPIO_PIN_7;
    gpio.Mode   = GPIO_MODE_INPUT;
    gpio.Pull   = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOG, &gpio);

    // PG6
    gpio.Pin    = GPIO_PIN_6;
    gpio.Mode   = GPIO_MODE_AF_PP;
    gpio.Pull   = GPIO_NOPULL;
    gpio.Speed  = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF10_OTG_FS;
    HAL_GPIO_Init(GPIOG, &gpio);

    // PG8
    gpio.Pin    = GPIO_PIN_8;
    gpio.Mode   = GPIO_MODE_OUTPUT_PP;
    gpio.Pull   = GPIO_NOPULL;
    gpio.Speed  = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOG, &gpio);

    // PC7
    gpio.Pin    = GPIO_PIN_7;
    gpio.Mode   = GPIO_MODE_AF_PP;
    gpio.Pull   = GPIO_NOPULL;
    gpio.Speed  = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF6_SPI3;
    HAL_GPIO_Init(GPIOC, &gpio);

    // PA8
    gpio.Pin    = GPIO_PIN_8;
    gpio.Mode   = GPIO_MODE_AF_PP;
    gpio.Pull   = GPIO_NOPULL;
    gpio.Speed  = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF0_MCO;
    HAL_GPIO_Init(GPIOA, &gpio);

    // PA10 PA11 PA12
    gpio.Pin    = GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
    gpio.Mode   = GPIO_MODE_AF_PP;
    gpio.Pull   = GPIO_NOPULL;
    gpio.Speed  = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF10_OTG_FS;
    HAL_GPIO_Init(GPIOA, &gpio);

    // PD3
    gpio.Pin    = GPIO_PIN_3;
    gpio.Mode   = GPIO_MODE_INPUT;
    gpio.Pull   = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOD, &gpio);

    // PB5
    gpio.Pin    = GPIO_PIN_5;
    gpio.Mode   = GPIO_MODE_AF_PP;
    gpio.Pull   = GPIO_NOPULL;
    gpio.Speed  = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF6_SPI3;
    HAL_GPIO_Init(GPIOB, &gpio);

    // PA0 - JOY_SEL
    gpio.Pin    = GPIO_PIN_0;
    gpio.Mode   = GPIO_MODE_IT_RISING;
    gpio.Pull   = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpio);
}

void init_timer()
{
    using hw::dev::timer;

    static_assert(hw::conf::PCLK2_FREQ / 1000 == 36'000);

    __HAL_RCC_TIM6_CLK_ENABLE();

    HAL_NVIC_SetPriority(TIM6_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM6_IRQn);

    timer.Instance          = TIM6;
    timer.Init.Prescaler    = (hw::conf::PCLK2_FREQ / 1000) - 1; // 1 kHz
    timer.Init.CounterMode  = TIM_COUNTERMODE_UP;
    timer.Init.Period       = motologger::conf::TIME_RESOLUTION - 1; // 100 ms
    if (HAL_TIM_Base_Init(&timer) != HAL_OK) {
        panic("timer: failed to initialize");
    }

    TIM_MasterConfigTypeDef master_conf;
    master_conf.MasterOutputTrigger = TIM_TRGO_RESET;
    master_conf.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&timer, &master_conf) != HAL_OK) {
        panic("timer: failed to initialize master configuration");
    }
}

void init_rtc()
{
    using hw::dev::rtc;

    static_assert(hw::conf::LSE_FREQ == 32'768);

    __HAL_RCC_RTC_ENABLE();

    rtc.Instance           = RTC;
    rtc.Init.HourFormat    = RTC_HOURFORMAT_24;
    rtc.Init.AsynchPrediv  = 127; // Configured for 32.768 kHz clock input
    rtc.Init.SynchPrediv   = 255;
    rtc.Init.OutPut        = RTC_OUTPUT_DISABLE;
    rtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
    rtc.Init.OutPutType    = RTC_OUTPUT_TYPE_OPENDRAIN;

    if (int err = HAL_RTC_Init(&rtc); err != HAL_OK) {
        panic("rtc: failed to initialize: %d", err);
    }
}

void init_can()
{
    using hw::dev::can;

    __HAL_RCC_CAN1_CLK_ENABLE();

    GPIO_InitTypeDef gpio;
    gpio.Pin        = GPIO_PIN_8 | GPIO_PIN_9;
    gpio.Mode       = GPIO_MODE_AF_PP;
    gpio.Pull       = GPIO_NOPULL;
    gpio.Speed      = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate  = GPIO_AF8_CAN1;
    HAL_GPIO_Init(GPIOB, &gpio);

    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);

    can.Instance                = CAN1;
    can.Init.TimeTriggeredMode  = DISABLE;
    can.Init.AutoBusOff         = DISABLE;
    can.Init.AutoWakeUp         = DISABLE;
    can.Init.AutoRetransmission = DISABLE;
    can.Init.ReceiveFifoLocked  = DISABLE;
    can.Init.TransmitFifoPriority = DISABLE;
    can.Init.Mode               = CAN_MODE_NORMAL;
    can.Init.SyncJumpWidth      = CAN_SJW_1TQ;
    can.Init.TimeSeg1           = CAN_BS1_7TQ;
    can.Init.TimeSeg2           = CAN_BS2_1TQ;
    can.Init.Prescaler          = 4;

    if (int err = HAL_CAN_Init(&can); err != HAL_OK) {
        panic("can: failed to initialize: %d", err);
    }

    CAN_FilterTypeDef filter;
    filter.FilterBank           = 0;
    filter.FilterMode           = CAN_FILTERMODE_IDMASK;
    filter.FilterScale          = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh         = 0x0000;
    filter.FilterIdLow          = 0x0000;
    filter.FilterMaskIdHigh     = 0x0000;
    filter.FilterMaskIdLow      = 0x0000;
    filter.FilterFIFOAssignment = 0;
    filter.FilterActivation     = ENABLE;
    filter.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&can, &filter) != HAL_OK) {
        panic("can: failed to initialize filter configuration");
    }
}

void init_i2c()
{
    using hw::dev::i2c;

    GPIO_InitTypeDef gpio;
    gpio.Pin        = GPIO_PIN_6 | GPIO_PIN_7;
    gpio.Mode       = GPIO_MODE_AF_OD;
    gpio.Pull       = GPIO_PULLUP;
    gpio.Speed      = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate  = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &gpio);

    __HAL_RCC_I2C1_CLK_ENABLE();

    i2c.Instance                = I2C1;
    i2c.Init.ClockSpeed         = 100000;
    i2c.Init.DutyCycle          = I2C_DUTYCYCLE_2;
    i2c.Init.OwnAddress1        = 0;
    i2c.Init.AddressingMode     = I2C_ADDRESSINGMODE_7BIT;
    i2c.Init.DualAddressMode    = I2C_DUALADDRESS_DISABLE;
    i2c.Init.OwnAddress2        = 0;
    i2c.Init.GeneralCallMode    = I2C_GENERALCALL_DISABLE;
    i2c.Init.NoStretchMode      = I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(&i2c) != HAL_OK) {
        panic("i2c: failed to initialize");
    }
}

void init_sd()
{
    using hw::dev::sd;

    __HAL_RCC_SDIO_CLK_ENABLE();

    GPIO_InitTypeDef gpio;
    gpio.Pin        = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11
                      | GPIO_PIN_12;
    gpio.Mode       = GPIO_MODE_AF_PP;
    gpio.Pull       = GPIO_NOPULL;
    gpio.Speed      = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate  = GPIO_AF12_SDIO;
    HAL_GPIO_Init(GPIOC, &gpio);

    gpio.Pin        = GPIO_PIN_2;
    gpio.Mode       = GPIO_MODE_AF_PP;
    gpio.Pull       = GPIO_NOPULL;
    gpio.Speed      = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate  = GPIO_AF12_SDIO;
    HAL_GPIO_Init(GPIOD, &gpio);

    sd.Instance             = SDIO;
    sd.Init.ClockEdge       = SDIO_CLOCK_EDGE_RISING;
    sd.Init.ClockBypass     = SDIO_CLOCK_BYPASS_DISABLE;
    sd.Init.ClockPowerSave  = SDIO_CLOCK_POWER_SAVE_DISABLE;
    sd.Init.BusWide         = SDIO_BUS_WIDE_1B;
    sd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
    sd.Init.ClockDiv        = 0;
}

#ifdef CONFIG_HAS_IMU
void init_imu()
{
    using drivers::lsm9ds1;
    using hw::dev::imu;

    imu.accel_scale = lsm9ds1::accel_scale::scale_4g;
    imu.accel_freq  = lsm9ds1::accel_freq::freq_10hz;
    imu.gyro_scale  = lsm9ds1::gyro_scale::scale_245dps;
    imu.gyro_freq   = lsm9ds1::gyro_freq::freq_15hz;
    imu.mag_scale   = lsm9ds1::mag_scale::scale_4g;
    imu.mag_freq    = lsm9ds1::mag_freq::freq_5hz;
    imu.init();
}
#else
void init_imu() {}
#endif

#ifdef CONFIG_HAS_UART
void init_uart()
{
    using hw::dev::uart;

    __HAL_RCC_USART2_CLK_ENABLE();

    GPIO_InitTypeDef gpio;
    gpio.Pin        = GPIO_PIN_2 | GPIO_PIN_3;
    gpio.Mode       = GPIO_MODE_AF_PP;
    gpio.Pull       = GPIO_PULLUP;
    gpio.Speed      = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate  = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &gpio);

    uart.Instance           = USART2;
    uart.Init.BaudRate      = 115200;
    uart.Init.WordLength    = UART_WORDLENGTH_8B;
    uart.Init.StopBits      = UART_STOPBITS_1;
    uart.Init.Parity        = UART_PARITY_NONE;
    uart.Init.Mode          = UART_MODE_TX_RX;
    uart.Init.HwFlowCtl     = UART_HWCONTROL_NONE;
    uart.Init.OverSampling  = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&uart) != HAL_OK) {
        panic("uart: failed to initialize");
    }
}
#endif

}

void hw::init()
{
    (void) HAL_Init();

    init_nvic();
    init_rcc();
    init_gpio();
    init_uart();
    init_timer();
    init_rtc();
    init_can();
    init_i2c();
    init_sd();
    init_imu();
}

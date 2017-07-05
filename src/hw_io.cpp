#include "hw.hpp"

void hw::io::imu_read(uint8_t addr, uint8_t* data, size_t size)
{
    using hw::dev::i2c;

    HAL_I2C_Master_Receive(&i2c, (addr & 0x7f) << 1, data,
                           (uint16_t) size, 100);
}

void hw::io::imu_write(uint16_t addr, uint8_t* data, size_t size)
{
    using hw::dev::i2c;

    HAL_I2C_Master_Transmit(&i2c, (addr & 0x7f) << 1, data,
                            (uint16_t) size, 100);
}

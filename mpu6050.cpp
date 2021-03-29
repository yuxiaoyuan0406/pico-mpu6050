#include "mpu6050.h"

/********************************************************/
/*          MPU-6050/MPU-6000 Register Map              */
/*          Check datasheet for details.                */
/*  https://datasheetspdf.com/datasheet/MPU6050.html    */
static const uint8_t PWR_MGMT_1 = 0x6B;

mpu6050::mpu6050(i2c_inst_t *i2c, uint sda, uint scl, int addr, uint baudrate):
_i2c_port(i2c), _sda_pin(sda), _scl_pin(scl), _baudrate(baudrate), _addr(addr)
{
    i2c_init(this->_i2c_port, this->_baudrate);
    gpio_set_function(this->_sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(this->_scl_pin, GPIO_FUNC_I2C);
    gpio_pull_up(this->_sda_pin);
    gpio_pull_up(this->_scl_pin);

    this->_reset();
}

mpu6050::~mpu6050()
{
}

void mpu6050::_reset()
{
    uint8_t buf[2] = {PWR_MGMT_1, 0x00};
    i2c_write_blocking(this->_i2c_port, this->_addr, buf, 2, false);
}

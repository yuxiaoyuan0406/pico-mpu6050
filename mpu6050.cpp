#include "mpu6050.h"

/********************************************************/
/*          MPU-6050/MPU-6000 Register Map              */
/*          Check datasheet for details.                */
/*  https://datasheetspdf.com/datasheet/MPU6050.html    */
typedef const uint8_t mpu_reg_t;

// Accelerometer Measurements
mpu_reg_t ACCEL_XOUT_H = 0x3B;
mpu_reg_t ACCEL_XOUT_L = 0x3C;
mpu_reg_t ACCEL_YOUT_H = 0x3D;
mpu_reg_t ACCEL_YOUT_L = 0x3E;
mpu_reg_t ACCEL_ZOUT_H = 0x3F;
mpu_reg_t ACCEL_ZOUT_L = 0x40;
// Temperature Measurement
mpu_reg_t TEMP_OUT_H = 0x41;
mpu_reg_t TEMP_OUT_L = 0x42;
// Gyroscope Measurements
mpu_reg_t GYRO_XOUT_H = 0x43;
mpu_reg_t GYRO_XOUT_L = 0x44;
mpu_reg_t GYRO_YOUT_H = 0x45;
mpu_reg_t GYRO_YOUT_L = 0x46;
mpu_reg_t GYRO_ZOUT_H = 0x47;
mpu_reg_t GYRO_ZOUT_L = 0x48;

// Power Management
mpu_reg_t PWR_MGMT_1 = 0x6B;

mpu6050::mpu6050(i2c_inst_t *i2c, uint sda, uint scl, int addr, uint baudrate) : _i2c_port(i2c), _sda_pin(sda), _scl_pin(scl), _baudrate(baudrate), _addr(addr)
{
    if (this->_i2c_port->hw->enable == 0)
    {
        i2c_init(this->_i2c_port, this->_baudrate);
        gpio_set_function(this->_sda_pin, GPIO_FUNC_I2C);
        gpio_set_function(this->_scl_pin, GPIO_FUNC_I2C);
        gpio_pull_up(this->_sda_pin);
        gpio_pull_up(this->_scl_pin);
    }
    this->_reset();
}

mpu6050::~mpu6050()
{
}

void mpu6050::read_raw(int16_t *accel, int16_t *gyro, int16_t *temp)
{
    uint8_t *buffer = new uint8_t[6];

    if (accel != nullptr)
    {
        this->_write_blocking(&ACCEL_XOUT_H, 1, true);
        this->_read_blocking(buffer, 6, false);

        for (int i = 0; i < 3; i++) {
            *(accel + i) = (*(buffer + i * 2) << 8 | *(buffer + i * 2 + 1));
        }
    }

    if(gyro != nullptr)
    {
        this->_write_blocking(&GYRO_XOUT_H, 1, true);
        this->_read_blocking(buffer, 6, false);

        for (int i = 0; i < 3; i++) {
            *(gyro + i) = (*(buffer + i * 2) << 8 | *(buffer + i * 2 + 1));
        }
    }

    if(temp != nullptr)
    {
        this->_write_blocking(&TEMP_OUT_H, 1, true);
        this->_read_blocking(buffer, 2, false);

        *temp = *buffer << 8 | *(buffer + 1);
    }

    delete[] buffer;
}

void mpu6050::_reset()
{
    uint8_t buf[2] = {PWR_MGMT_1, 0x00};
    this->_write_blocking(buf, 2, false);
}

#include "../defines.h"
#include "mpu6050.h"

#ifdef _DEBUG
#include <stdio.h>
#endif // _DEBUG

/********************************************************/
/*          MPU-6050/MPU-6000 Register Map              */
/*          Check datasheet for details.                */
/*  https://datasheetspdf.com/datasheet/MPU6050.html    */
typedef const uint8_t mpu_reg_t;

// Configurations
mpu_reg_t GYRO_CONFIG = 0x1B;
mpu_reg_t ACCEL_CONFIG = 0x1C;

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
mpu_reg_t USER_CTRL  = 0x6A;
mpu_reg_t PWR_MGMT_1 = 0x6B;
mpu_reg_t PWR_MGMT_2 = 0x6C;

mpu6050::mpu6050(i2c_inst_t *i2c, uint sda, uint scl, int addr, uint baudrate) : _i2c_port(i2c), _sda_pin(sda), _scl_pin(scl), _baudrate(baudrate), _addr(addr), _accel_full_scale(accel_2g), _gyro_full_scale(gyro_250_degrees)
{
    if (this->_i2c_port->hw->enable == 0)
    {
#ifdef _DEBUG
        printf("I2C port initializing...\r\n");
#endif // _DEBUG
        i2c_init(this->_i2c_port, this->_baudrate);
        gpio_set_function(this->_sda_pin, GPIO_FUNC_I2C);
        gpio_set_function(this->_scl_pin, GPIO_FUNC_I2C);
        gpio_pull_up(this->_sda_pin);
        gpio_pull_up(this->_scl_pin);
    }
#ifdef _DEBUG
    else
    {
        printf("I2C port already initialized.\r\n");
    }
#endif // _DEBUG

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

        for (int i = 0; i < 3; i++)
        {
            *(accel + i) = (*(buffer + i * 2) << 8 | *(buffer + i * 2 + 1));
        }
    }

    if (gyro != nullptr)
    {
        this->_write_blocking(&GYRO_XOUT_H, 1, true);
        this->_read_blocking(buffer, 6, false);

        for (int i = 0; i < 3; i++)
        {
            *(gyro + i) = (*(buffer + i * 2) << 8 | *(buffer + i * 2 + 1));
        }
    }

    if (temp != nullptr)
    {
        this->_write_blocking(&TEMP_OUT_H, 1, true);
        this->_read_blocking(buffer, 2, false);

        *temp = *buffer << 8 | *(buffer + 1);
    }

    delete[] buffer;
}

inline int mpu6050::_write_blocking(const uint8_t *src, size_t len, bool nostop) { return i2c_write_blocking(this->_i2c_port, this->_addr, src, len, nostop); }

inline int mpu6050::_read_blocking(uint8_t *dst, size_t len, bool nostop) { return i2c_read_blocking(this->_i2c_port, this->_addr, dst, len, nostop); }

inline void mpu6050::_write_register(uint8_t reg, uint8_t val)
{
#ifdef _DEBUG
    printf("Writing register 0x%02X value 0x%02X\r\n", reg, val);
#endif // _DEBUG
    uint8_t msg[2] = {reg, val};
    // this->_write_blocking(&reg, 1, true);
    // this->_write_blocking(&val, 1, false);
    this->_write_blocking(msg, 2, false);
}

inline void mpu6050::_reset()
{
    this->_write_register(PWR_MGMT_1, 0x01);
    this->_write_register(PWR_MGMT_2, 0x00);
}

void mpu6050::set_gyro_full_scale(gyro_full_scale_sel sel)
{
    this->_gyro_full_scale = sel;
    uint8_t val = 0xFF & (sel << 3);
#ifdef _DEBUG
    printf("Gyro config. 0x%02X\r\n", val);
#endif // _DEBUG
    this->_write_register(GYRO_CONFIG, val);
    this->_reset();
}

void mpu6050::set_accel_full_scale(accel_full_scale_sel sel)
{
    this->_accel_full_scale = sel;
    uint8_t val = 0xFF & (sel << 3);
#ifdef _DEBUG
    printf("Accel config. 0x%02X\r\n", val);
#endif // _DEBUG
    this->_write_register(ACCEL_CONFIG, val);
    this->_reset();
}

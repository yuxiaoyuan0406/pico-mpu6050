#pragma once

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/types.h"

#if !defined(i2c_default) || !defined(PICO_DEFAULT_I2C_SDA_PIN) || !defined(PICO_DEFAULT_I2C_SCL_PIN)
#warning The mpu6050 class requires a board with I2C pins. If not defined, please specified port, sda and scl pin when calling the constructor.
#endif

static const uint default_baudrate = 400 * 1000;
static const int default_addr = 0x68;

class mpu6050
{
public:
    enum gyro_full_scale_sel
    {
        gyro_250_degrees = 0,
        gyro_500_degrees = 1,
        gyro_1000_degrees = 2,
        gyro_2000_degrees = 3,
    };
    enum accel_full_scale_sel
    {
        accel_2g = 0,
        accel_4g = 1,
        accel_8g = 2,
        accel_16g = 3,
    };

    /*! \brief  Create a mpu6050 object on a specified i2c port. 
    * \param i2c I2C port which mpu connected to. default = i2c_default
    * \param sda GPIO number to sda. default = PICO_DEFAULT_I2C_SDA_PIN
    * \param scl GPIO number to scl. default = PICO_DEFAULT_I2C_SCL_PIN
    * \param addr Address of mpu. default = 0x68
    * \param baudrate I2C baudrate. default = 400*1000
    */
    mpu6050(i2c_inst_t *i2c = i2c_default,
            uint sda = PICO_DEFAULT_I2C_SDA_PIN,
            uint scl = PICO_DEFAULT_I2C_SCL_PIN,
            int addr = default_addr,
            uint baudrate = default_baudrate);
    ~mpu6050();
    /*! \brief  Read raw data from mpu.
    * \param accel Acceleration buffer. Will not read if NULL.
    * \param gyro Gyroscope buffer. Will not read if NULL.
    * \param temp Temperature buffer. Will not read if NULL.
    */
    void read_raw(int16_t *accel, int16_t *gyro, int16_t *temp);

    /*! \brief  Select the full scale range of the gyroscope outputs.
    * \param sel full scale selection, enum
    */
    void set_gyro_full_scale(gyro_full_scale_sel sel);
    /*! \brief  Select the full scale range of the accelerometer outputs.
    * \param sel full scale selection, enum
    */
    void set_accel_full_scale(accel_full_scale_sel sel);

    /*! \brief Attempt to write specified number of bytes to the address of the mpu, blocking
    *
    * \param src Pointer to data to send
    * \param len Length of data in bytes to send
    * \param nostop  If true, master retains control of the bus at the end of the transfer (no Stop is issued),
    *           and the next transfer will begin with a Restart rather than a Start.
    * \return Number of bytes written, or PICO_ERROR_GENERIC if address not acknowledged, no device present.
    */
    int write_blocking(const uint8_t *scr, size_t len, bool nostop);
    /*! \brief  Attempt to read specified number of bytes from the address of the mpu, blocking
    *
    * \param dst Pointer to buffer to receive data
    * \param len Length of data in bytes to receive
    * \param nostop  If true, master retains control of the bus at the end of the transfer (no Stop is issued),
    *           and the next transfer will begin with a Restart rather than a Start.
    * \return Number of bytes read, or PICO_ERROR_GENERIC if address not acknowledged, no device present.
    */
    int read_blocking(uint8_t *dst, size_t len, bool nostop);

private:
    i2c_inst_t *_i2c_port;
    uint _sda_pin, _scl_pin, _baudrate;
    int _addr;
    gyro_full_scale_sel _gyro_full_scale;
    accel_full_scale_sel _accel_full_scale;
    void _reset();

    /*! \brief Attempt to write specified number of bytes to this->_addr, blocking
    *
    * \param src Pointer to data to send
    * \param len Length of data in bytes to send
    * \param nostop  If true, master retains control of the bus at the end of the transfer (no Stop is issued),
    *           and the next transfer will begin with a Restart rather than a Start.
    * \return Number of bytes written, or PICO_ERROR_GENERIC if address not acknowledged, no device present.
    */
    int _write_blocking(const uint8_t *src, size_t len, bool nostop);
    /*! \brief  Attempt to read specified number of bytes from this->_addr, blocking
    *
    * \param dst Pointer to buffer to receive data
    * \param len Length of data in bytes to receive
    * \param nostop  If true, master retains control of the bus at the end of the transfer (no Stop is issued),
    *           and the next transfer will begin with a Restart rather than a Start.
    * \return Number of bytes read, or PICO_ERROR_GENERIC if address not acknowledged, no device present.
    */
    int _read_blocking(uint8_t *dst, size_t len, bool nostop);

    /*! \brief Write a single byte to a specified register
    * \param reg the address of the register
    * \param val the byte to write
    */
    void _write_register(uint8_t reg, uint8_t val);
};

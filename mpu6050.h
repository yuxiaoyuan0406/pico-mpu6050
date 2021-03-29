#pragma once

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/types.h"

#if !defined(i2c_default) || !defined(PICO_DEFAULT_I2C_SDA_PIN) || !defined(PICO_DEFAULT_I2C_SCL_PIN)
    #warning The mpu6050 class requires a board with I2C pins. If not defined, please specified port, sda and scl pin when calling the constructor. 
#else

static const uint default_baudrate = 400 * 1000;
static const int default_addr = 0x68;

class mpu6050
{
private:
    i2c_inst_t *_i2c_port;
    uint _sda_pin, _scl_pin, _baudrate;
    int _addr;
    void _reset();

    /*! \brief Attempt to write specified number of bytes to this->_addr, blocking
    *
    * \param src Pointer to data to send
    * \param len Length of data in bytes to send
    * \param nostop  If true, master retains control of the bus at the end of the transfer (no Stop is issued),
    *           and the next transfer will begin with a Restart rather than a Start.
    * \return Number of bytes written, or PICO_ERROR_GENERIC if address not acknowledged, no device present.
    */
    inline int _write_blocking(const uint8_t *src, size_t len, bool nostop) { return i2c_write_blocking(this->_i2c_port, this->_addr, src, len, nostop); }

/*! \brief  Attempt to read specified number of bytes from this->_addr, blocking
 *
 * \param dst Pointer to buffer to receive data
 * \param len Length of data in bytes to receive
 * \param nostop  If true, master retains control of the bus at the end of the transfer (no Stop is issued),
 *           and the next transfer will begin with a Restart rather than a Start.
 * \return Number of bytes read, or PICO_ERROR_GENERIC if address not acknowledged, no device present.
 */
    inline int _read_blocking(uint8_t *dst, size_t len, bool nostop) { i2c_read_blocking(this->_i2c_port, this->_addr, dst, len, nostop); }
public:
    mpu6050(i2c_inst_t *i2c = i2c_default,
            uint sda = PICO_DEFAULT_I2C_SDA_PIN,
            uint scl = PICO_DEFAULT_I2C_SCL_PIN,
            int addr = default_addr, 
            uint baudrate = default_baudrate);
    ~mpu6050();
    void read_raw(int16_t *accel, int16_t *gyro, int16_t *temp);
};

#endif
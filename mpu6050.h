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
public:
    mpu6050(i2c_inst_t *i2c = i2c_default,
            uint sda = PICO_DEFAULT_I2C_SDA_PIN,
            uint scl = PICO_DEFAULT_I2C_SCL_PIN,
            int addr = default_addr, 
            uint baudrate = default_baudrate);
    ~mpu6050();
};

#endif
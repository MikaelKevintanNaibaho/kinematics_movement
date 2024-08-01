#ifndef MOCK_I2C_H
#define MOCK_I2C_H

#include "i2c_interface.h"
#include <stdint.h>

static int mock_i2c_open(const char *device);
static void mock_i2c_close(void);
static int mock_set_slave_address(uint8_t addr);
static int mock_write_byte(uint8_t reg, uint8_t val);
static uint8_t mock_read_byte(uint8_t reg);

void set_mock_i2c(void);
#endif // MOCK_I2C_H

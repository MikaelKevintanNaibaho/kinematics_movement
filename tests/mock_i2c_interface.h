#ifndef MOCK_I2C_INTERFACE_H
#define MOCK_I2C_INTERFACE_H

#include "i2c_interface.h"
#include <stdint.h>

// Function prototypes for the mock functions
void set_mock_i2c_interface(void);
void reset_mock_i2c_counters(void);
void set_mock_i2c_read_byte_val(uint8_t reg, uint8_t val);

// Functions to access mock call counts
int get_mock_i2c_open_count(void);
int get_mock_i2c_set_slave_address_count(void);
int get_mock_i2c_close_count(void);
int get_mock_i2c_write_byte_count(void);
int get_mock_i2c_read_byte_count(void);

const char *get_mock_i2c_open_device(void);

uint8_t get_mock_i2c_set_slave_addr(void);
uint8_t get_mock_i2c_write_byte_reg(int index);
uint8_t get_mock_i2c_write_byte_val(int index);
uint8_t get_mock_i2c_read_byte_reg(int index);
uint8_t get_mock_i2c_read_byte_val(int index);

#endif // MOCK_I2C_INTERFACE_H

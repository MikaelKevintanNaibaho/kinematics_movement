#include "i2c_interface.h"

// Mock implementation functions
static int mock_i2c_open(const char *device)
{
    // Mock behavior for opening the I2C device
    return 0;
}

static int mock_i2c_set_slave_address(uint8_t addr)
{
    // Mock behavior for setting the slave address
    return 0;
}

static void mock_i2c_close()
{
    // Mock behavior for closing the I2C device
}

static int mock_i2c_write_byte(uint8_t reg, uint8_t val)
{
    // Mock behavior for writing a byte
    return 0;
}

static uint8_t mock_i2c_read_byte(uint8_t reg)
{
    // Mock behavior for reading a byte
    return 0;
}

// Define the mock I2C interface
I2CInterface mock_i2c_interface = { .open = mock_i2c_open,
                                    .set_slave_address = mock_i2c_set_slave_address,
                                    .close = mock_i2c_close,
                                    .write_byte = mock_i2c_write_byte,
                                    .read_byte = mock_i2c_read_byte };

// Function to set the mock I2C interface
void set_mock_i2c_interface()
{
    set_i2c_interface(&mock_i2c_interface);
}

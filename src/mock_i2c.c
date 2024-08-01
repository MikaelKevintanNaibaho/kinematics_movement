#include "mock_i2c.h"
#include "i2c_interface.h"

static int mock_i2c_open(const char *device)
{
    return 0;
}

static void mock_i2c_close(void) { }

static int mock_i2c_set_slave_address(uint8_t addr)
{
    return 0;
}

static int mock_write_byte(uint8_t reg, uint8_t val)
{
    return 0;
}

static uint8_t mock_read_byte(uint8_t reg)
{
    return 0xFF;
}

static I2CInterface mock_i2c_interface = { .open = mock_i2c_open,
                                           .close = mock_i2c_close,
                                           .set_slave_address = mock_i2c_set_slave_address,
                                           .read_byte = mock_read_byte,
                                           .write_byte = mock_write_byte };
void set_mock_i2c(void)
{
    set_i2c_interface(&mock_i2c_interface);
}

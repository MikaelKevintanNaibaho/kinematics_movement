#include <unity/unity.h>
#include "test_i2c.h"
#include "mock_i2c_interface.h"

#define I2C_DEVICE "/dev/i2c-1"
#define PCA9685_SLAVE_ADDR 0x40

void test_i2c_open(void)
{
    I2CInterface *iface = get_i2c_interface();
    iface->open(I2C_DEVICE);

    TEST_ASSERT_EQUAL(1, get_mock_i2c_open_count());
    TEST_ASSERT_EQUAL(I2C_DEVICE, get_mock_i2c_open_device());
}
void test_i2c_set_slave_address(void)
{
    // Test if the set_slave_address function is called
    I2CInterface *iface = get_i2c_interface();
    iface->set_slave_address(PCA9685_SLAVE_ADDR);

    TEST_ASSERT_EQUAL(1, get_mock_i2c_set_slave_address_count());
    TEST_ASSERT_EQUAL(PCA9685_SLAVE_ADDR, get_mock_i2c_set_slave_addr());
}

void test_i2c_write_byte(void)
{
    // Test if the write_byte function is called
    I2CInterface *iface = get_i2c_interface();
    iface->write_byte(0x00, 0xFF);

    TEST_ASSERT_EQUAL(1, get_mock_i2c_write_byte_count());
    TEST_ASSERT_EQUAL(0x00, get_mock_i2c_write_byte_reg(0));
    TEST_ASSERT_EQUAL(0xFF, get_mock_i2c_write_byte_val(0));
}

void test_i2c_read_byte(void)
{
    // Test if the read_byte function is called and returns the expected value
    set_mock_i2c_read_byte_val(0x00, 0xAA);

    I2CInterface *iface = get_i2c_interface();
    uint8_t value = iface->read_byte(0x00);

    TEST_ASSERT_EQUAL(1, get_mock_i2c_read_byte_count());
    TEST_ASSERT_EQUAL(0x00, get_mock_i2c_read_byte_reg(0));
    TEST_ASSERT_EQUAL(0xAA, value);
}

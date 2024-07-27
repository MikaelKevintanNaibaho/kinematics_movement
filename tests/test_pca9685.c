#include "test_pca9685.h"
#include "i2c_interface.h"
#include "mock_i2c_interface.h"
#include "pca9685.h"
#include <stdint.h>
#include <unity/unity.h>

#define TEST_ASSERT_EQUAL_WITH_MESSAGE(expected, actual, msg)                                      \
    do {                                                                                           \
        if ((expected) != (actual)) {                                                              \
            printf("%s: Expected %d but got %d\n", (msg), (expected), (actual));                   \
            TEST_FAIL();                                                                           \
        }                                                                                          \
    } while (0)

// Usage
void test_pca9685_init(void)
{
    TEST_ASSERT_EQUAL_WITH_MESSAGE(0, pca9685_init(get_i2c_interface()), "pca9685_init failed");

    TEST_ASSERT_EQUAL_WITH_MESSAGE(2, get_mock_i2c_open_count(), "I2C open call count mismatch");

    TEST_ASSERT_EQUAL_STRING(I2C_DEVICE, get_mock_i2c_open_device());

    TEST_ASSERT_EQUAL_WITH_MESSAGE(1, get_mock_i2c_set_slave_address_count(),
                                   "I2C set_slave_address call count mismatch");
    TEST_ASSERT_EQUAL(PCA9685_SLAVE_ADDR, get_mock_i2c_set_slave_addr());

    TEST_ASSERT_EQUAL_WITH_MESSAGE(6, get_mock_i2c_write_byte_count(),
                                   "I2C write_byte call count mismatch");

    TEST_ASSERT_EQUAL_WITH_MESSAGE(2, get_mock_i2c_close_count(), "I2C close call count mismatch");
}

void test_set_pwm_freq(void)
{
    set_pwm_freq(get_i2c_interface(), 50);
    TEST_ASSERT_EQUAL_WITH_MESSAGE(1, get_mock_i2c_open_count(), "I2C open call count mismatch");
    TEST_ASSERT_EQUAL_STRING(I2C_DEVICE, get_mock_i2c_open_device());
    TEST_ASSERT_EQUAL_WITH_MESSAGE(4, get_mock_i2c_write_byte_count(),
                                   "I2C write_byte call count mismatch");
    TEST_ASSERT_EQUAL_WITH_MESSAGE(1, get_mock_i2c_close_count(), "I2C cloe call count mismatch");
}
void test_set_pwm(void)
{
    int channel = 0;
    set_pwm(get_i2c_interface(), channel, 0, 100);

    // Assert open call count and device
    TEST_ASSERT_EQUAL_WITH_MESSAGE(1, get_mock_i2c_open_count(), "I2C open call count mismatch");
    TEST_ASSERT_EQUAL_STRING(I2C_DEVICE, get_mock_i2c_open_device());

    // Assert write call count and check each register value
    TEST_ASSERT_EQUAL_WITH_MESSAGE(4, get_mock_i2c_write_byte_count(),
                                   "I2C write_byte call count mismatch");

    // Assert the first write to ON_L register
    TEST_ASSERT_EQUAL_WITH_MESSAGE(channel0_ON_L + channel_MULTIPLIER * (channel - 1),
                                   get_mock_i2c_write_byte_reg(0), "Register mismatch for ON_L");
    TEST_ASSERT_EQUAL_WITH_MESSAGE(0 & 0xFF, get_mock_i2c_write_byte_val(0),
                                   "Value mismatch for ON_L");

    // Assert the second write to ON_H register
    TEST_ASSERT_EQUAL_WITH_MESSAGE(channel0_ON_L + channel_MULTIPLIER * (channel - 1) + 1,
                                   get_mock_i2c_write_byte_reg(1), "Register mismatch for ON_H");
    TEST_ASSERT_EQUAL_WITH_MESSAGE(0 >> 8, get_mock_i2c_write_byte_val(1),
                                   "Value mismatch for ON_H");

    // Assert the third write to OFF_L register
    TEST_ASSERT_EQUAL_WITH_MESSAGE(channel0_OFF_L + channel_MULTIPLIER * (channel - 1),
                                   get_mock_i2c_write_byte_reg(2), "Register mismatch for OFF_L");
    TEST_ASSERT_EQUAL_WITH_MESSAGE(100 & 0xFF, get_mock_i2c_write_byte_val(2),
                                   "Value mismatch for OFF_L");

    // Assert the fourth write to OFF_H register
    TEST_ASSERT_EQUAL_WITH_MESSAGE(channel0_OFF_L + channel_MULTIPLIER * (channel - 1) + 1,
                                   get_mock_i2c_write_byte_reg(3), "Register mismatch for OFF_H");
    TEST_ASSERT_EQUAL_WITH_MESSAGE(100 >> 8, get_mock_i2c_write_byte_val(3),
                                   "Value mismatch for OFF_H");

    // Assert close call count
    TEST_ASSERT_EQUAL_WITH_MESSAGE(1, get_mock_i2c_close_count(), "I2C close call count mismatch");
}

void test_set_pwm_duty(void)
{
    int channel = 0;
    // Call the function to test
    set_pwm_duty(get_i2c_interface(), channel, 100);

    // Assert open call count and device
    TEST_ASSERT_EQUAL_WITH_MESSAGE(1, get_mock_i2c_open_count(), "I2C open call count mismatch");
    TEST_ASSERT_EQUAL_STRING(I2C_DEVICE, get_mock_i2c_open_device());

    // Assert write call count and check each register value
    TEST_ASSERT_EQUAL_WITH_MESSAGE(4, get_mock_i2c_write_byte_count(),
                                   "I2C write_byte call count mismatch");

    // Assert the first write to ON_L register
    TEST_ASSERT_EQUAL_WITH_MESSAGE(channel0_ON_L + channel_MULTIPLIER * (channel - 1),
                                   get_mock_i2c_write_byte_reg(0), "Register mismatch for ON_L");
    TEST_ASSERT_EQUAL_WITH_MESSAGE(0 & 0xFF, get_mock_i2c_write_byte_val(0),
                                   "Value mismatch for ON_L");

    // Assert the second write to ON_H register
    TEST_ASSERT_EQUAL_WITH_MESSAGE(channel0_ON_L + channel_MULTIPLIER * (channel - 1) + 1,
                                   get_mock_i2c_write_byte_reg(1), "Register mismatch for ON_H");
    TEST_ASSERT_EQUAL_WITH_MESSAGE(0 >> 8, get_mock_i2c_write_byte_val(1),
                                   "Value mismatch for ON_H");

    // Assert the third write to OFF_L register
    TEST_ASSERT_EQUAL_WITH_MESSAGE(channel0_OFF_L + channel_MULTIPLIER * (channel - 1),
                                   get_mock_i2c_write_byte_reg(2), "Register mismatch for OFF_L");
    TEST_ASSERT_EQUAL_WITH_MESSAGE(100 & 0xFF, get_mock_i2c_write_byte_val(2),
                                   "Value mismatch for OFF_L");

    // Assert the fourth write to OFF_H register
    TEST_ASSERT_EQUAL_WITH_MESSAGE(channel0_OFF_L + channel_MULTIPLIER * (channel - 1) + 1,
                                   get_mock_i2c_write_byte_reg(3), "Register mismatch for OFF_H");
    TEST_ASSERT_EQUAL_WITH_MESSAGE(100 >> 8, get_mock_i2c_write_byte_val(3),
                                   "Value mismatch for OFF_H");

    // Assert close call count
    TEST_ASSERT_EQUAL_WITH_MESSAGE(1, get_mock_i2c_close_count(), "I2C close call count mismatch");
}

void test_get_pwm(void)
{
    uint8_t channel = 0;
    int expected_value = 0x1234;

    uint8_t register1 = (channel0_OFF_L + channel_MULTIPLIER * (channel - 1));
    uint8_t register2 = (channel0_OFF_L + channel_MULTIPLIER * (channel - 1) + 1);

    set_mock_i2c_read_byte_val(register1, 0x34);
    set_mock_i2c_read_byte_val(register2, 0x12);

    int result = get_pwm(get_i2c_interface(), channel);

    TEST_ASSERT_EQUAL_WITH_MESSAGE(1, get_mock_i2c_open_count(), "I2C open call count mismatch");
    TEST_ASSERT_EQUAL_STRING(I2C_DEVICE, get_mock_i2c_open_device());
    TEST_ASSERT_EQUAL_WITH_MESSAGE(2, get_mock_i2c_read_byte_count(),
                                   "I2C read_byte call count mismatch");
    TEST_ASSERT_EQUAL_WITH_MESSAGE(register1, get_mock_i2c_read_byte_reg(0),
                                   "first register mismatch");
    TEST_ASSERT_EQUAL_WITH_MESSAGE(0x34, get_mock_i2c_read_byte_val(0),
                                   "firt read_byte value mismatch");
    TEST_ASSERT_EQUAL_WITH_MESSAGE(register2, get_mock_i2c_read_byte_reg(1),
                                   "second register mismatch");
    TEST_ASSERT_EQUAL_WITH_MESSAGE(0x12, get_mock_i2c_read_byte_val(1),
                                   "second read_byte value mismatch");

    TEST_ASSERT_EQUAL_WITH_MESSAGE(1, get_mock_i2c_close_count(), "I2c close call count mismatch");
    TEST_ASSERT_EQUAL_WITH_MESSAGE(expected_value, result,
                                   "result does not match the expected value");
}

void test_set_pwm_angle(void)
{
    // Define some constants for the test
    uint8_t channel = 0;
    int angle = 90; // Midpoint angle
    int expected_pulse_width =
        MIN_PULSE_WIDTH + ((MAX_PULSE_WIDTH - MIN_PULSE_WIDTH) * angle / 180);

    // Reset mock counters
    reset_mock_i2c_counters();

    // Call the function to test
    set_pwm_angle(get_i2c_interface(), channel, angle);

    // Assert write call count and check each register value
    TEST_ASSERT_EQUAL_WITH_MESSAGE(4, get_mock_i2c_write_byte_count(),
                                   "I2C write_byte call count mismatch");

    // Assert the first write to ON_L register (mock write index 0)
    TEST_ASSERT_EQUAL_WITH_MESSAGE(channel0_ON_L + channel_MULTIPLIER * (channel - 1),
                                   get_mock_i2c_write_byte_reg(0), "Register mismatch for ON_L");
    TEST_ASSERT_EQUAL_WITH_MESSAGE(0 & 0xFF, get_mock_i2c_write_byte_val(0),
                                   "Value mismatch for ON_L");

    // Assert the second write to ON_H register (mock write index 1)
    TEST_ASSERT_EQUAL_WITH_MESSAGE(channel0_ON_L + channel_MULTIPLIER * (channel - 1) + 1,
                                   get_mock_i2c_write_byte_reg(1), "Register mismatch for ON_H");
    TEST_ASSERT_EQUAL_WITH_MESSAGE(0 >> 8, get_mock_i2c_write_byte_val(1),
                                   "Value mismatch for ON_H");

    // Assert the third write to OFF_L register (mock write index 2)
    TEST_ASSERT_EQUAL_WITH_MESSAGE(channel0_OFF_L + channel_MULTIPLIER * (channel - 1),
                                   get_mock_i2c_write_byte_reg(2), "Register mismatch for OFF_L");
    TEST_ASSERT_EQUAL_WITH_MESSAGE(expected_pulse_width & 0xFF, get_mock_i2c_write_byte_val(2),
                                   "Value mismatch for OFF_L");

    // Assert the fourth write to OFF_H register (mock write index 3)
    TEST_ASSERT_EQUAL_WITH_MESSAGE(channel0_OFF_L + channel_MULTIPLIER * (channel - 1) + 1,
                                   get_mock_i2c_write_byte_reg(3), "Register mismatch for OFF_H");
    TEST_ASSERT_EQUAL_WITH_MESSAGE(expected_pulse_width >> 8, get_mock_i2c_write_byte_val(3),
                                   "Value mismatch for OFF_H");
}

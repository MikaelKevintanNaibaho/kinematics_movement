#include "i2c_interface.h"
#include "pca9685.h"
#include <stdint.h>

// Use the global i2c_iface for I2C operations

int pca9685_init(I2CInterface *i2c_iface)
{
    if (i2c_iface->open(I2C_DEVICE) < 0) {
        return -1;
    }

    if (i2c_iface->set_slave_address(PCA9685_SLAVE_ADDR) < 0) {
        i2c_iface->close();
        return -1;
    }

    if (i2c_iface->write_byte(MODE1, 0x00) < 0 || i2c_iface->write_byte(MODE2, SUBADR3) < 0) {
        i2c_iface->close();
        return -1;
    }

    set_pwm_freq(i2c_iface, 50);

    i2c_iface->close();
    return 0;
}

void set_pwm_freq(I2CInterface *i2c_iface, int freq)
{
    uint8_t prescale_val = (uint8_t)((CLOCK_FREQ / 4096 * freq) - 1);
    if (i2c_iface->open(I2C_DEVICE) < 0) {
        return;
    }

    i2c_iface->write_byte(MODE1, 0x10); // Sleep
    i2c_iface->write_byte(PRE_SCALE, prescale_val);
    i2c_iface->write_byte(MODE1, 0x80); // Restart
    i2c_iface->write_byte(MODE2, 0x04); // Totem pole (default)

    i2c_iface->close();
}

void set_pwm_duty(I2CInterface *i2c_iface, uint8_t channel, int value)
{
    set_pwm(i2c_iface, channel, 0, value);
}

void set_pwm(I2CInterface *i2c_iface, uint8_t channel, int on_value, int off_value)
{
    if (i2c_iface->open(I2C_DEVICE) < 0) {
        return;
    }

    i2c_iface->write_byte(channel0_ON_L + channel_MULTIPLIER * (channel - 1), on_value & 0xFF);
    i2c_iface->write_byte(channel0_ON_L + channel_MULTIPLIER * (channel - 1) + 1, on_value >> 8);
    i2c_iface->write_byte(channel0_OFF_L + channel_MULTIPLIER * (channel - 1), off_value & 0xFF);
    i2c_iface->write_byte(channel0_OFF_L + channel_MULTIPLIER * (channel - 1) + 1, off_value >> 8);

    i2c_iface->close();
}

int get_pwm(I2CInterface *i2c_iface, uint8_t channel)
{
    int channel_value = 0;
    if (i2c_iface->open(I2C_DEVICE) < 0) {
        return 0;
    }

    channel_value = i2c_iface->read_byte(channel0_OFF_L + channel_MULTIPLIER * (channel - 1));
    channel_value |=
        (i2c_iface->read_byte(channel0_OFF_L + channel_MULTIPLIER * (channel - 1) + 1) << 8);

    i2c_iface->close();
    return channel_value;
}

void set_pwm_angle(I2CInterface *i2c_iface, uint8_t channel, int angle)
{
    if (angle < 0) {
        angle = 0;
    } else if (angle > 180) {
        angle = 180;
    }

    int pulse_width = MIN_PULSE_WIDTH + ((MAX_PULSE_WIDTH - MIN_PULSE_WIDTH) * angle / 180);

    set_pwm_duty(i2c_iface, channel, pulse_width);
}

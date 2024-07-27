#ifndef PCA9685_H
#define PCA9685_H

#include <stdint.h>
#include "i2c_interface.h"
#define I2C_DEVICE "/dev/i2c-2"
#define PCA9685_SLAVE_ADDR 0x40
#define MODE1 0x00 // Mode  register  1
#define MODE2 0x01 // Mode  register  2
#define SUBADR1 0x02 // I2C-bus subaddress 1
#define SUBADR2 0x03 // I2C-bus subaddress 2
#define SUBADR3 0x04 // I2C-bus subaddress 3
#define ALLCALLADR 0x05 // channel All Call I2C-bus address
#define channel0 0x6 // channel0 start register
#define channel0_ON_L 0x6 // channel0 output and brightness control byte 0
#define channel0_ON_H 0x7 // channel0 output and brightness control byte 1
#define channel0_OFF_L 0x8 // channel0 output and brightness control byte 2
#define channel0_OFF_H 0x9 // channel0 output and brightness control byte 3
#define channel_MULTIPLIER 4 // For the other 15 channels
#define ALLchannel_ON_L 0xFA // load all the channeln_ON registers, byte 0 (turn 0-7 channels on)
#define ALLchannel_ON_H 0xFB // load all the channeln_ON registers, byte 1 (turn 8-15 channels on)
#define ALLchannel_OFF_L 0xFC // load all the channeln_OFF registers, byte 0 (turn 0-7 channels off)
#define ALLchannel_OFF_H                                                                           \
    0xFD // load all the channeln_OFF registers, byte 1 (turn 8-15 channels off)
#define PRE_SCALE 0xFE // prescaler for output frequency
#define CLOCK_FREQ 25000000.0 // 25MHz default osc clock
#define ANGLE_RANGE 180
#define MIN_PULSE_WIDTH 400
#define MAX_PULSE_WIDTH 2600

int pca9685_init(I2CInterface *i2c_iface);
void set_pwm_freq(I2CInterface *i2c_iface, int freq);
void set_pwm_duty(I2CInterface *i2c_iface, uint8_t channel, int value);
void set_pwm(I2CInterface *i2c_iface, uint8_t channel, int on_value, int off_value);
int get_pwm(I2CInterface *i2c_iface, uint8_t channel);
void set_pwm_angle(I2CInterface *i2c_iface, uint8_t channel, int angle);

#endif // PCA9685_H

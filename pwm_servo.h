#ifndef PWM_SERVO_H
#define PWM_SERVO_H


#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>

#define I2C_DEVICE "/dev/12c-1"
#define PCA9685 0x40
#define MODE1_REG 0x00
#define PRE_SCALE 0xFE
#define LED0_ON_L 0x06
#define NUM_CHANNELS 16

extern int i2c_fd;

int open_i2_device(const char* device);
int init_pca9685();
void set_servo_position(int channel, int position);



#endif /*PWM_SERVO_H*/
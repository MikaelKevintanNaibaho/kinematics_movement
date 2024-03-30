#include "pwm_servo.h"

int i2c_fd;

int main(void) {

    if (init_pca9685(I2C_DEVICE) != 0) {
        close(i2c_fd);  // Close I2C device before exiting
        return -1;
    }

    set_servo_position(0, 1500);  // Set servo on channel 0 to midpoint

    close(i2c_fd);  // Close I2C device after use
    return 0;
}
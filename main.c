#include "pwm_servo.h"

int i2c_fd;

int main(void) 
{
    if (open_i2_device(I2C_DEVICE) != 0) {
        return -1;
    }

    if (init_pca9685() != 0 ){
        return -1;
    }

    set_servo_position(0, 1500);

    close(i2c_fd);
    return 0;
}
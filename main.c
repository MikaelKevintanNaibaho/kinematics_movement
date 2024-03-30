#include "pwm_servo.h"

int i2c_fd;

int main(void) {

    PCA9685_init();

    set_pwm_freq(1000);

    set_pwm_duty(1, 1024.0);

    int pwmValue = get_pwm(0);


    printf("Current PWM value for channel 1: %d\n", pwmValue);

    // Close I2C device
    close(i2c_fd);
    return 0;
}
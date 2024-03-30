#include "pwm_servo.h"

int i2c_fd;

int main(void) {
        // Initialize PCA9685 (void function, no return value)
    PCA9685_init();

    // Set PWM frequency to 1000Hz (void function, no return value)
    set_pwm_freq(1000);

    // Set PWM duty cycle for channel 1 to 50% (void function, no return value)
    set_pwm_duty(1, 2048);

    // Get current PWM value for channel 1
    int pwmValue = get_pwm(1);
    printf("Current PWM value for channel 1: %d\n", pwmValue);

    // Close I2C device
    close(i2c_fd);

    return 0;
}
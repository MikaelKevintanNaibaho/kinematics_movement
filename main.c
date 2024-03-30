#include <stdio.h>
#include <unistd.h>
#include "pwm_servo.h"

#define MIN_DUTY_CYCLE 6.25   // Duty cycle for 0 degrees (500μs pulse width at 120Hz)
#define MAX_DUTY_CYCLE 31.25  // Duty cycle for 180 degrees (2500μs pulse width at 120Hz)

int main() {
    // Initialize PCA9685
    PCA9685_init();

    // Set PWM frequency to 120Hz
    set_pwm_freq(120);

    // Move servo to 0 degrees (500μs pulse width)
    set_pwm_duty(1, MIN_DUTY_CYCLE);
    usleep(1000000); // Wait for 1 second

    // Move servo to 180 degrees (2500μs pulse width)
    set_pwm_duty(1, MAX_DUTY_CYCLE);
    usleep(1000000); // Wait for 1 second

    // Close I2C device
    close(i2c_fd);

    return 0;
}
#include <stdio.h>
#include <unistd.h>
#include "pwm_servo.h"

#define MIN_DUTY_CYCLE 150  // Duty cycle for 0 degrees (approx. 5%)
#define MAX_DUTY_CYCLE 600  // Duty cycle for 180 degrees (approx. 10%)
#define DELAY_US 1000000    // Delay between movements in microseconds (1 second)

int main() {
    // Initialize PCA9685
    PCA9685_init();

    // Set PWM frequency to 50Hz for standard servo motors
    set_pwm_freq(50);

    while (1) {
        // Move servo to 0 degrees
        set_pwm_duty(1, MIN_DUTY_CYCLE);
        usleep(DELAY_US); // Wait for 1 second

        // Move servo to 180 degrees
        set_pwm_duty(1, MAX_DUTY_CYCLE);
        usleep(DELAY_US); // Wait for 1 second
    }

    // Close I2C device (Unreachable in this loop, added for completeness)
    close(i2c_fd);

    return 0;
}

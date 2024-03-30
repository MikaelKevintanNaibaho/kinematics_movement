#include <stdio.h>
#include <unistd.h>
#include "pwm_servo.h"

#define MIN_DUTY_CYCLE 500   // Duty cycle for 0 degrees (approx. 500μs pulse width at 1000Hz)
#define MAX_DUTY_CYCLE 2500  // Duty cycle for 180 degrees (approx. 2500μs pulse width at 1000Hz)
#define DELAY_US 1000000   // Delay between movements in microseconds (1 second)

int main() {
    // Initialize PCA9685
    PCA9685_init();

    // Set PWM frequency to 1000Hz (within the specified 50-330Hz range)
    set_pwm_freq(50);

    while (1) {
        // Move servo to 0 degrees (500μs pulse width)
        set_pwm_duty(1, MIN_DUTY_CYCLE);
        usleep(DELAY_US); // Wait for 1 second

        // Move servo to 180 degrees (2500μs pulse width)
        set_pwm_duty(1, MAX_DUTY_CYCLE);
        usleep(DELAY_US); // Wait for 1 second
    }

    // Close I2C device (Unreachable in this loop, added for completeness)
    close(i2c_fd);

    return 0;
}

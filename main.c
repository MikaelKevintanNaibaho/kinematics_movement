#include "pwm_servo.h"
#include <stdio.h>
#include <unistd.h>

int i2c_fd;


#define MIN_DUTY_CYCLE 150  // Minimum duty cycle for servo (approx. 0 degrees)
#define MAX_DUTY_CYCLE 600  // Maximum duty cycle for servo (approx. 180 degrees)
#define SWEEP_DELAY_MS 10   // Delay between PWM duty cycle changes in milliseconds

int main() {
    // Initialize PCA9685
    PCA9685_init();

    // Set PWM frequency to 50Hz for standard servo motors
    set_pwm_freq(50);

    // Sweep motion
    int dutyCycle = MIN_DUTY_CYCLE; // Start from minimum duty cycle
    int direction = 1; // 1 for increasing duty cycle, -1 for decreasing

    while (1) {
        // Set PWM duty cycle
        set_pwm_duty(1, dutyCycle);

        // Print current position (approximate angle)
        int angle = map(dutyCycle, MIN_DUTY_CYCLE, MAX_DUTY_CYCLE, 0, 180);
        printf("Current angle: %d degrees\n", angle);

        // Increment or decrement duty cycle based on direction
        dutyCycle += direction;

        // Check if reached end points, change direction
        if (dutyCycle >= MAX_DUTY_CYCLE || dutyCycle <= MIN_DUTY_CYCLE) {
            direction *= -1; // Change direction
            usleep(SWEEP_DELAY_MS * 1000); // Delay before changing direction
        }

        usleep(SWEEP_DELAY_MS * 1000); // Delay between duty cycle changes
    }

    // Close I2C device
    close(i2c_fd);

    return 0;
}

// Utility function to map a value from one range to another
int map(int x, int in_min, int in_max, int out_min, int out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
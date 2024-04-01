#include <stdio.h>
#include <unistd.h>
#include "pwm_servo.h"

// PWM frequency and period
#define PWM_FREQUENCY 300   // PWM frequency in Hz
#define PWM_PERIOD_US (1000000 / PWM_FREQUENCY)  // PWM period in microseconds

// Servo angle pulse widths in microseconds
#define ANGLE_0_PULSE_WIDTH_US 500    // Pulse width for 0 degrees
#define ANGLE_90_PULSE_WIDTH_US 1000  // Pulse width for 90 degrees
#define ANGLE_180_PULSE_WIDTH_US 2000 // Pulse width for 180 degrees

int main() {
    // Initialize PCA9685
    PCA9685_init();

    // Calculate duty cycles based on pulse widths and PWM period
    float duty_cycle_0 = (float)ANGLE_0_PULSE_WIDTH_US / PWM_PERIOD_US * 100;
    float duty_cycle_90 = (float)ANGLE_90_PULSE_WIDTH_US / PWM_PERIOD_US * 100;
    float duty_cycle_180 = (float)ANGLE_180_PULSE_WIDTH_US / PWM_PERIOD_US * 100;

    // Convert duty cycles to PWM range (0-4095 for 12-bit resolution)
    int pwm_duty_cycle_0 = (int)(duty_cycle_0 / 100 * 4095);
    int pwm_duty_cycle_90 = (int)(duty_cycle_90 / 100 * 4095);
    int pwm_duty_cycle_180 = (int)(duty_cycle_180 / 100 * 4095);

    // Loop through angles continuously
    while (1) {
        set_pwm_duty(1, pwm_duty_cycle_0);     // 0 degrees
        usleep(1000000); // Wait for 1 second
        set_pwm_duty(1, pwm_duty_cycle_90);    // 90 degrees
        usleep(1000000); // Wait for 1 second
        set_pwm_duty(1, pwm_duty_cycle_180);   // 180 degrees
        usleep(1000000); // Wait for 1 second
    }

    // Close I2C device (Unreachable in this loop, added for completeness)
    close(i2c_fd);

    return 0;
}

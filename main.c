#include <stdio.h>
#include <unistd.h>
#include "pwm_servo.h"


// Define constants for servo operation
#define MIN_PULSE_WIDTH 130 // Minimum pulse width in microseconds for max speed
#define MAX_PULSE_WIDTH 2500 // Maximum pulse width in microseconds
#define PWM_FREQUENCY 50 // PWM frequency in Hz

// Other code remains unchanged

int main() {
    PCA9685_init();
    set_pwm_freq(PWM_FREQUENCY); // Set PWM frequency
    
    // Calculate duty cycle for max speed (for 180 degrees)
    int duty_cycle = (MIN_PULSE_WIDTH * 4096) / (1000000 / PWM_FREQUENCY);
    
    while(1){
            // Set PWM duty cycle for servo
        set_pwm_duty(1, duty_cycle);
    }


    // Continue with your program logic or testing

    return 0;
}


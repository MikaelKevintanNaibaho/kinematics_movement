#include <stdio.h>
#include <unistd.h>
#include "pwm_servo.h"
#include "ik.h"

#define PWM_FREQ 50
#define SERVO_CHANNEL_1 1
#define SERVO_CHANNEL_2 2
#define SERVO_CHANNEL_3 3

int main(void)
{
    PCA9685_init();

    
    set_pwm_angle(SERVO_CHANNEL_1, 0, PWM_FREQ);
    set_pwm_angle(SERVO_CHANNEL_2, 0, PWM_FREQ);
    set_pwm_angle(SERVO_CHANNEL_3, 0, PWM_FREQ);

    return 0;
}
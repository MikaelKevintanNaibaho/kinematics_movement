#include <stdio.h>
#include <unistd.h>
#include "pwm_servo.h"
#include "ik.h"

#define SERVO_CHANNEL_1 1
#define SERVO_CHANNEL_2 2
#define SERVO_CHANNEL_3 3
#define SERVO_CHANNEL_4 4
#define SERVO_CHANNEL_5 5
#define SERVO_CHANNEL_6 6
#define SERVO_CHANNEL_7 7
#define SERVO_CHANNEL_8 8
#define SERVO_CHANNEL_9 9
#define SERVO_CHANNEL_10 10
#define SERVO_CHANNEL_11 11
#define SERVO_CHANNEL_12 12

int main(void)
{
    PCA9685_init();

    
    set_pwm_angle(SERVO_CHANNEL_1, 0, PWM_FREQ);
    set_pwm_angle(SERVO_CHANNEL_2, 0, PWM_FREQ);
    set_pwm_angle(SERVO_CHANNEL_3, 0, PWM_FREQ);
    set_pwm_angle(SERVO_CHANNEL_4, 0, PWM_FREQ);
    set_pwm_angle(SERVO_CHANNEL_5, 0, PWM_FREQ);
    set_pwm_angle(SERVO_CHANNEL_6, 0, PWM_FREQ);
    set_pwm_angle(SERVO_CHANNEL_7, 0, PWM_FREQ);
    set_pwm_angle(SERVO_CHANNEL_8, 0, PWM_FREQ);
    set_pwm_angle(SERVO_CHANNEL_9, 0, PWM_FREQ);
    set_pwm_angle(SERVO_CHANNEL_10, 0, PWM_FREQ);
    set_pwm_angle(SERVO_CHANNEL_11, 0, PWM_FREQ);
    set_pwm_angle(SERVO_CHANNEL_12, 0, PWM_FREQ);

    return 0;
}
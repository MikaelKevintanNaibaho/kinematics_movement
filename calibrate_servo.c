#include "calibrate_servo.h"

void set_zero(void) {
    set_pwm_angle(SERVO_CHANNEL_1, 0);
    set_pwm_angle(SERVO_CHANNEL_2, 0);
    set_pwm_angle(SERVO_CHANNEL_3, 0);
    set_pwm_angle(SERVO_CHANNEL_4, 0);
    set_pwm_angle(SERVO_CHANNEL_5, 0);
    set_pwm_angle(SERVO_CHANNEL_6, 0);
    set_pwm_angle(SERVO_CHANNEL_7, 0);
    set_pwm_angle(SERVO_CHANNEL_8, 0);
    set_pwm_angle(SERVO_CHANNEL_9, 0);
    set_pwm_angle(SERVO_CHANNEL_10, 0);
    set_pwm_angle(SERVO_CHANNEL_11, 0);
    set_pwm_angle(SERVO_CHANNEL_12, 0);
}



int main(void) {
    PCA9685_init();

    set_zero();

    return 0;
}
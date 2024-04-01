#include <stdio.h>
#include <unistd.h>
#include "pwm_servo.h"

int main(void)
{
    PCA9685_init();

    while(1){
        set_pwm_angle(2, 0, 50);
        int pwm_value = get_pwm(2);
        printf("pwm: %d", pwm_value);

        sleep(1);

        set_pwm_angle(2, 90, 50);
        pwm_value = get_pwm(2);
        printf("pwm: %d", pwm_value);

        sleep(1);

        set_pwm_angle(2, 180, 50);
        pwm_value = get_pwm(2);
        printf("pwm: %d", pwm_value);

        sleep(1);
    }


    return 0;
}

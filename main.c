#include <stdio.h>
#include <unistd.h>
#include "pwm_servo.h"

int main(void)
{
    PCA9685_init();


    while(1){
        set_pwm_angle(1, 0, 50);

        sleep(1);

        set_pwm_angle(1, 90, 50);

        sleep(1);

        set_pwm_angle(1, 170, 50);

        sleep(1);
    }


    return 0;
}

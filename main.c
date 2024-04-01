#include <stdio.h>
#include <unistd.h>
#include "pwm_servo.h"

int main(void)
{
    PCA9685_init();

    set_pwm_freq(50);

    int angle;
    for(angle =  0; angle <= ANGLE_RANGE; angle ++)
    {
        set_servo_angle(1, angle, 50);
    }

    return 0;
}

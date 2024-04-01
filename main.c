#include <stdio.h>
#include <unistd.h>
#include "pwm_servo.h"

int main(void)
{
    PCA9685_init();

    set_pwm_angle(1, 90, 200);

    return 0;
}

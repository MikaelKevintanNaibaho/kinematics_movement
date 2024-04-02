#include <stdio.h>
#include <unistd.h>
#include "pwm_servo.h"
#include "ik.h"



int main(void)
{
    PCA9685_init();

    set_pwm_angle(1, INITIAL_ANGLE1, 50);
    set_pwm_angle(2, INITIAL_ANGLE2, 50);
    set_pwm_angle(3, INITIAL_ANGLE3, 50);

    sleep(5);

    float starx = 0.0, starty = 0.0, startz = 0.0;
    float endx = 0.0, endy = 50.0, endz = 0.0;
    int steps = 50;
    int freq = 50;

    move_leg(starx, starty, startz, endx, endy, endz, steps, freq);


    return 0;
}

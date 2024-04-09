#include "ik.h"
#include "pwm_servo.h"

int main(void){
    PCA9685_init();

    SpiderLeg leg;
    float target[3] = {100.0, 100.0, 0.0};

    inverse_kinematics(&leg, target);

    sleep(2);
    target[0] += 0.0;
    target[1] += 0.0;
    target[2] += 50.0;

    inverse_kinematics(&leg, target);

    return 0;
}
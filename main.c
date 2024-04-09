#include "ik.h"

int main(void){
    PCA9685_init();

    SpiderLeg leg;
    float target[3] = {100.0, 100.0, 0.0};

    inverse_kinematics(&leg, target);

    sleep(2);
    target[0] += 50.0;
    target[0] += 50.0;

    inverse_kinematics(&leg, target);

    return 0;
}
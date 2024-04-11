#include"pwm_servo.h"
#include "ik.h"

int main(void) {
    PCA9685_init();

    SpiderLeg leg;
    float initial_angle[3] = {45, 150, 130};
    set_angles(&leg, initial_angle);
    forward_kinematics(&leg, initial_angle);

    printf("x = %.2f, y = %.2f, z = %.2f\n", leg.joints[3][0], leg.joints[3][1], leg.joints[3][2]);

}
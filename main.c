#include"pwm_servo.h"
#include "ik.h"

int main(void) {
    PCA9685_init();

    SpiderLeg leg;
    leg.theta1 = 0;
    leg.theta2 = 0;
    leg.theta3 = 0;
    float initial_angle[3] = {0, 130, 130};
    // set_angles(&leg, initial_angle);
    move_to_angle(&leg, initial_angle, 100);
    forward_kinematics(&leg, initial_angle);

    printf("x = %.2f, y = %.2f, z = %.2f\n", leg.joints[3][0], leg.joints[3][1], leg.joints[3][2]);

}
#include"pwm_servo.h"
#include "ik.h"

int main(void) {
    PCA9685_init();

    SpiderLeg leg;
    float initial_angle[3] = {45, 90, 90};
    set_angles(&leg, initial_angle);
    forward_kinematics(&leg, initial_angle);
    printf("x = %.2f, y = %.2f, z = %.2f\n", leg.joints[3][0], leg.joints[3][1], leg.joints[3][2]); 

    sleep(2);

    float target_position[3] = {0, 100, 0};
    inverse_kinematics(&leg, target_position);
    // move_to_angle(&leg, initial_angle, 1000);
    // forward_kinematics(&leg, initial_angle);

    printf("x = %.2f, y = %.2f, z = %.2f\n", leg.joints[3][0], leg.joints[3][1], leg.joints[3][2]);

}
#include"pwm_servo.h"
#include "ik.h"

int main(void) {
    PCA9685_init();

    SpiderLeg leg;
    float initial_angle[3] = {0, 100, 100};
    set_angles(&leg, initial_angle);
    forward_kinematics(&leg, initial_angle);
    printf("x = %.2f, y = %.2f, z = %.2f\n", leg.joints[3][0], leg.joints[3][1], leg.joints[3][2]); 

    sleep(2);

    

    float keyframes[4][3] = {
        {0, 0, 30},
        {50.0, 0, 0},
        {0, 0, 0},
        {-50, 0, 30}
    };

    // Move leg through keyframes
    for (int i = 0; i < 4; i++) {
        inverse_kinematics(&leg, keyframes[i]);
        printf("x = %.2f, y = %.2f, z = %.2f\n", leg.joints[3][0], leg.joints[3][1], leg.joints[3][2]); 
        // Wait for a short duration before moving to the next keyframe
        sleep(1);
    }

    printf("x = %.2f, y = %.2f, z = %.2f\n", leg.joints[3][0], leg.joints[3][1], leg.joints[3][2]);

}
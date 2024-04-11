#include "ik.h"
#include "pwm_servo.h"

int perform_ik(SpiderLeg *leg, float target[3]) {
    IK_ErrorCode error_code = inverse_kinematics(leg, target);
    if(error_code != IK_SUCCESS) {
        handle_error(error_code);
        return 0;
    }
    return 1; // Return 1 to indicate success
}

int main(void) {
    PCA9685_init();

    SpiderLeg leg;
    float initial_angle[3] = {45, 150, 130};
    forward_kinematics(&leg, initial_angle);

    printf("x = %.2f, y = %.2f, z = %.2f\n", leg.joints[3][0], leg.joints[3][1], leg.joints[3][2]);

}
#include "pwm_servo.h"
#include "ik.h"

int main(void) {
    PCA9685_init();

    SpiderLeg leg;
    gsl_matrix *intermediate_link[NUM_LINKS];
    for (int i = 0; i < NUM_LINKS; i++) {
        intermediate_link[i] = gsl_matrix_alloc(4, 4);
    }
    float initial_angle[3] = {0, 100, 100};
    set_angles(&leg, initial_angle);

    sleep(2);
    forward_kinematics(&leg, initial_angle, intermediate_link); // Removed the '&' before intermediate_link

    float target_positions[3] = {0.0, 0.0, 50.0};

    inverse_kinematics(&leg, target_positions, intermediate_link);
    return 0;
}
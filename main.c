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
    for (int i = 0; i < 3; i++){
        float target_positions[3] = {0.0, 0.0, 50.0};
        inverse_kinematics(&leg, target_positions, intermediate_link);
        sleep(1);
        target_positions[2] += -50;
        sleep(1);
    }

    for (int i = 0; i < 3; i++){
        float target_positions[3] = {0.0, 50.0, 00.0};
        inverse_kinematics(&leg, target_positions, intermediate_link);
        sleep(1);
        target_positions[1] += -50;
        sleep(1);
    }

    for (int i = 0; i < 3; i++){
        float target_positions[3] = {50.0, 0.0, 0.0};
        inverse_kinematics(&leg, target_positions, intermediate_link);
        sleep(1);
        target_positions[0] += -50;
        sleep(1);
    }
    

    return 0;
}
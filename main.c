#include "pwm_servo.h"
#include "ik.h"
#include "move.h"

int main(void) {
    PCA9685_init();

    SpiderLeg leg;
    gsl_matrix *intermediate_link[NUM_LINKS];
    for (int i = 0; i < NUM_LINKS; i++) {
        intermediate_link[i] = gsl_matrix_alloc(4, 4);
    }
    float initial_angle[3] = {0, 90, 90};
    set_angles(&leg, initial_angle);

    sleep(2);
    forward_kinematics(&leg, initial_angle, intermediate_link); // Removed the '&' before intermediate_link

    while(1){
        walk_forward(&leg, intermediate_link, 100, 50, NUM_POINTS);
    }
    // struct bezier2d stright_back;
    // bezier2d_init(&stright_back);

    // generate_stright_back_trajectory(&stright_back, &leg, stride_length);
    // update_leg_position_with_velocity(&stright_back, 3, &leg, intermediate_link);
    
    for (int i = 0; i < NUM_LINKS; i++) {
        gsl_matrix_free(intermediate_link[i]);
    }
    return 0;
}
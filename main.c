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
    float initial_angle[3] = {0, 130, 130};
    set_angles(&leg, initial_angle);

    sleep(2);
    forward_kinematics(&leg, initial_angle, intermediate_link); // Removed the '&' before intermediate_link

    // float target[3] = {-50.0, 0.0, 0.0};
    // inverse_kinematics(&leg, target, intermediate_link);

    // float angle[3] = {0, 90, 90};
    // move_to_angle(&leg, angle, 100);

    struct bezier2d curve;
    bezier2d_init(&curve);

    float stride_length = 100.0;
    float swing_high = 50.0;
    generate_walk_trajectory(&curve, &leg, stride_length, swing_high);

    save_trajectory_points(&curve, "trajectory.dat", 50);


    int num_points = 60.0;
    update_leg_position_with_velocity(&curve, num_points, &leg, intermediate_link);
    usleep(100000);

    struct bezier2d stright_back;
    bezier2d_init(&stright_back);

    generate_stright_back_trajectory(&stright_back, &leg, stride_length);
    update_leg_position_with_velocity(&stright_back, num_points, &leg, intermediate_link);

    for (int i = 0; i < NUM_LINKS; i++) {
        gsl_matrix_free(intermediate_link[i]);
    }
    return 0;
}
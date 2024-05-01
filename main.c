#include "ik.h"
#include "move.h"
#include "pwm_servo.h"

int main(void)
{
    // Initialize PCA9685 if necessary
    PCA9685_init();

    initialize_all_legs();

    // Set initial angles using forward kinematics
    for (int i = 0; i < NUM_LEGS; i++) {
        printf("Setting initial angles for Leg %s (Position %d):\n", legs[i]->name,
               leg_positions[i]);
        set_angles(legs[i], stance_angles[i]);
        forward_kinematics(legs[i], stance_angles[i], leg_positions[i]);
        printf("----------------------------\n");
    }

    sleep(2);

    struct bezier2d curve[NUM_LEGS];
    for (int i = 0; i < NUM_LEGS; i++) {
        bezier2d_init(&curve[i]);
        if (leg_positions[i] == KANAN_BELAKANG || leg_positions[i] == KIRI_BELAKANG) {
            generate_walk_back_leg(&curve[i], legs[i], STRIDE_LENGTH, SWING_HEIGTH,
                                   leg_positions[i]);
        } else {
            generate_walk_trajectory(&curve[i], legs[i], STRIDE_LENGTH, SWING_HEIGTH,
                                     leg_positions[i]);
        }
        print_trajectory(&curve[i], 30);
    }
    // crawl_gait(legs, leg_positions);
    while (1) {
        // update_leg_wave_gait(curve, NUM_POINTS, legs, leg_positions);
        update_leg_crawl_gait(curve, NUM_POINTS, legs, leg_positions);
    }

    // ripple_gait(legs, leg_positions);

    return 0;
}

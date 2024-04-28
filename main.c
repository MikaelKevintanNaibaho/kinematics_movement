#include "ik.h"
#include "move.h"
#include "pwm_servo.h"

int main(void)
{
    // Initialize PCA9685 if necessary
    PCA9685_init();

    // Declare instances for each leg
    SpiderLeg leg_kiri_depan;
    SpiderLeg leg_kiri_belakang;
    SpiderLeg leg_kanan_belakang;
    SpiderLeg leg_kanan_depan;

    // Declare an array of pointers to SpiderLeg instances
    SpiderLeg *legs[NUM_LEGS] = { &leg_kiri_depan, &leg_kiri_belakang, &leg_kanan_belakang,
                                  &leg_kanan_depan };

    // Pass the array of pointers to SpiderLeg instances to the
    // initialize_all_legs function
    initialize_all_legs(legs);

    // Define parameters for walking gait
    // Number of points for trajectory interpolation

    // Define leg positions
    LegPosition leg_positions[NUM_LEGS] = { KIRI_DEPAN, KIRI_BELAKANG, KANAN_BELAKANG,
                                            KANAN_DEPAN };

    // Define initial angles for the stance position
    float stance_angles[NUM_LEGS][3] = { { 45.0, 130.0, 130.0 },
                                         { 45.0, 130.0, 130.0 },
                                         { 45.0, 130.0, 130.0 },
                                         { 45.0, 130.0, 130.0 } };

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

    for (int  i = 0; i < NUM_LEGS; i++) {
        bezier2d_init(&curve[i]);
        if (leg_positions[i] == KANAN_BELAKANG || leg_positions[i] == KIRI_BELAKANG){
          generate_walk_trajectory(&curve[i], legs[i], -STRIDE_LENGTH, SWING_HEIGTH, leg_positions[i]);
        } else {
          generate_walk_trajectory(&curve[i], legs[i], STRIDE_LENGTH, SWING_HEIGTH, leg_positions[i]);
        }

    }
    //
    // crawl_gait(legs, leg_positions);
    while (1) {
      update_leg_wave_gait(&curve, NUM_POINTS, legs, leg_positions);
    }
    // while (1) {
    //     // wave_gait(legs, STRIDE_LENGTH, SWING_HEIGTH, leg_positions);
    //     wave_gait(legs, leg_positions);
    // }

    // ripple_gait(legs, leg_positions);

    return 0;
}

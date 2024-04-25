#include "pwm_servo.h"
#include "ik.h"
#include "move.h"

int main(void) {
    // Initialize PCA9685 if necessary
    PCA9685_init();

    // Declare instances for each leg
    SpiderLeg leg_kiri_depan;
    SpiderLeg leg_kiri_belakang;
    SpiderLeg leg_kanan_belakang;
    SpiderLeg leg_kanan_depan;

    // Declare an array of pointers to SpiderLeg instances
    SpiderLeg *legs[NUM_LEGS] = {&leg_kiri_depan, &leg_kiri_belakang, &leg_kanan_belakang, &leg_kanan_depan};

    // Pass the array of pointers to SpiderLeg instances to the initialize_all_legs function
    initialize_all_legs(legs);

    // Define parameters for walking gait
    float stride_length = 50.0; // Adjust as needed
    float swing_height = 20.0;   // Adjust as needed
    int num_points = 50;         // Number of points for trajectory interpolation

    // Define leg positions
    LegPosition leg_positions[NUM_LEGS] = {KIRI_DEPAN, KIRI_BELAKANG, KANAN_BELAKANG, KANAN_DEPAN};

    // Define initial angles for the stance position
    float stance_angles[NUM_LEGS][3] = {
        {45.0, 130.0, 130.0},
        {45.0, 130.0, 130.0},
        {45.0, 130.0, 130.0},
        {45.0, 130.0, 130.0}
    };

    // Set initial angles using forward kinematics
    for (int i = 0; i < NUM_LEGS; i++) {
        printf("Setting initial angles for Leg %s (Position %d):\n", legs[i]->name, leg_positions[i]);
        set_angles(legs[i], stance_angles[i]);
        forward_kinematics(legs[i], stance_angles[i], leg_positions[i]);
        printf("----------------------------\n");
    }

    sleep(2);
    while (1){
        // Call walk forward function
        walk_forward(legs, stride_length, swing_height, num_points, leg_positions);
        sleep(1);
    }
    

    return 0;
}

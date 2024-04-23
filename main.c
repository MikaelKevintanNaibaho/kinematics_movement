#include "pwm_servo.h"
#include "ik.h"
#include "move.h"

int main(void) {
    PCA9685_init();

    SpiderLeg leg_kiri_depan;
    SpiderLeg leg_kiri_belakang;
    SpiderLeg leg_kanan_belakang;
    SpiderLeg leg_kanan_depan;

    // Declare an array of pointers to SpiderLeg instances
    SpiderLeg *legs[NUM_LEGS] = {&leg_kiri_depan, &leg_kiri_belakang, &leg_kanan_belakang, &leg_kanan_depan};

    // Pass the array of pointers to SpiderLeg instances to the initialize_all_legs function
    initialize_all_legs(legs);

    float angles[NUM_LEGS][3] = {
        {45.0, 130.0, 130.0},
        {45.0, 130.0, 130.0},
        {45.0, 130.0, 130.0},
        {45.0, 130.0, 130.0}
    };

    printf("----------------------------\n");

    // Define target positions
    float target_positions[NUM_LEGS][3] = {
        {-130.96, 84.96, -117.0},  // Example target position for leg_kiri_depan
        {-130.96, 84.96, -117.0}, // Example target position for leg_kiri_belakang
        {-130.96, -84.96, -117.0},// Example target position for leg_kanan_belakang
        {130.96, 84.96, -117.0}  // Example target position for leg_kanan_depan
    };

    // Define leg positions
    LegPosition leg_positions[NUM_LEGS] = {KIRI_DEPAN, KIRI_BELAKANG, KANAN_BELAKANG, KANAN_DEPAN};

    
    float offset_angle[NUM_LEGS] = {0.0, -90.0, -180.0, -270.0};

    // Pass the address of each leg and its respective angles to the set_angles function
    for (int i = 0; i < NUM_LEGS; i++) {
        set_angles(legs[i], angles[i]);
    }

    // Call forward kinematics for each leg
    for (int i = 0; i < NUM_LEGS; i++) {
        printf("Leg %s (Position %d):\n", legs[i]->name, leg_positions[i]);
        forward_kinematics(legs[i], angles[i], leg_positions[i]);
        printf("----------------------------\n");
    }

    sleep(2);

    // Call inverse kinematics for each leg with the corresponding target position and leg position
    for (int i = 0; i < NUM_LEGS; i++) {
        printf("INVERSE KINEMATICS\n\n\n");
        printf("Leg %s (Position %d):\n", legs[i]->name, leg_positions[i]);
        inverse_kinematics(legs[i], target_positions[i], leg_positions[i]);
        printf("----------------------------\n");
    }

    return 0;
}

#include <stdio.h>
#include "ik.h"

int main() {
    PCA9685_init();
    // Initialize intermediate matrices for DH transformation
    gsl_matrix *intermediate_matrices[NUM_LINKS];
    for (int i = 0; i < NUM_LINKS; i++) {
        intermediate_matrices[i] = gsl_matrix_alloc(4, 4);
    }

    // Define arrays to store leg instances and joint angles for each leg
    SpiderLeg legs[NUM_LEGS];
    float leg_angles[NUM_LEGS][3] = {
        {0.0, 130.0, 130.0},   // Angles for KANAN_DEPAN
        {0.0, 130.0, 130.0},   // Angles for KANAN_BELAKANG
        {0.0, 130.0, 130.0},   // Angles for KIRI_BELAKANG
        {0.0, 130.0, 130.0}    // Angles for KIRI_DEPAN
    };

    // Iterate over each leg
    for (int i = 0; i < NUM_LEGS; i++) {
        // Create leg instance
        legs[i] = create_leg((LegPosition)i);

        // Set the joint angles for the leg
        float *angles = leg_angles[i];
        set_angles(&legs[i], angles, (LegPosition)i); // Pass the leg position

        // Perform forward kinematics for the leg
        forward_kinematics(&legs[i], angles, intermediate_matrices);

        // Get the current position of the leg's tip
        float *current_position = get_target(&legs[i]);

        // Set the target position to move 100mm along the x-axis
        float target_position[3] = {current_position[0] + 100.0, current_position[1], current_position[2]};

        // Perform inverse kinematics to move the leg to the target position
        inverse_kinematics(&legs[i], target_position, intermediate_matrices, (LegPosition)i);
    }

    // Free intermediate matrices
    for (int i = 0; i < NUM_LINKS; i++) {
        gsl_matrix_free(intermediate_matrices[i]);
    }

    return 0;
}

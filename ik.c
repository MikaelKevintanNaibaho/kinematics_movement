#include "ik.h"

void calculate_ik(float x, float y, float z, float* angles)
{
    angles[0] = atan2(y, x);

    float dist_xy = sqrt(x*x + y*y) - COXA_LENGTH;

    float dist_xyz = sqrt(dist_xy*dist_xy + z*z);

    float alpha = acos((FEMUR_LENGTH*FEMUR_LENGTH + TIBIA_LENGTH*TIBIA_LENGTH - dist_xyz*dist_xyz) / (2 * FEMUR_LENGTH *TIBIA_LENGTH));

    float beta = acos ((dist_xy*dist_xyz + FEMUR_LENGTH*FEMUR_LENGTH - TIBIA_LENGTH*TIBIA_LENGTH) / (2 * dist_xyz * FEMUR_LENGTH));

    angles[1] = PI/2 - alpha - atan2(z, dist_xy);

    angles[2] = PI - beta;

    if (isnan(angles[1])) {
        angles[1] = INITIAL_ANGLE2; // Replace INITIAL_ANGLE_2 with the initial angle for joint 2
    }

    if (isnan(angles[2])) {
        angles[2] = INITIAL_ANGLE3; // Replace INITIAL_ANGLE_3 with the initial angle for joint 3
    }

    if (isnan(angles[0])) {
        angles[0] = INITIAL_ANGLE1;
    }

     // Print the calculated angles
    printf("Calculated Joint Angles:\n");
    printf("Angle 1: %f\n", angles[0]);
    printf("Angle 2: %f\n", angles[1]);
    printf("Angle 3: %f\n", angles[2]);
}

void move_leg(float startx, float starty, float startz, float endx, float endy, float endz, int steps, int freq)
{
    float current_x = startx;
    float current_y = starty;
    float current_z = startz;

    float step_x = (endx - startx) / steps;
    float step_y = (endy - starty) / steps;
    float step_z = (endz - startz) / steps;

    float joint_angles[3];

    for (int i = 0; i <= steps; i++){
        calculate_ik(current_x, current_y, current_z, joint_angles);

        set_pwm_angle(1, joint_angles[0], freq);
        set_pwm_angle(2, joint_angles[1], freq);
        set_pwm_angle(3, joint_angles[2], freq);

        current_x += step_x;
        current_y += step_y;
        current_z += step_z; // Corrected incrementing z axis

        usleep(1000000 / freq);
    }
}

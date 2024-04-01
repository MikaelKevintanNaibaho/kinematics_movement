#include "ik.h"

void calculate_ik(float x, float y, float z, float* angles)
{
    angles[0] = atan2(y, x);

    float dist_xy = sqrt(x*x + y*y) - COXA_LENGTH;

    float dist_xyz = sqrt(dist_xy*dist_xy + z*z);

    float alpha = acos((FEMUR_LEGTH*FEMUR_LEGTH + TIBIA_LEGTH*TIBIA_LEGTH - dist_xyz*dist_xyz) / (2 * FEMUR_LEGTH *TIBIA_LEGTH));

    float beta = acos ((dist_xy*dist_xyz +FEMUR_LEGTH*FEMUR_LEGTH - TIBIA_LEGTH) / (2 * dist_xyz * FEMUR_LEGTH));

    angles[1] = PI/2 - alpha - atan2(z, dist_xyz);

    angles[2] = PI - beta;
}


void move_leg(float startx, float starty, float startz, float endx, float endy, float endz, int steps, int freq)
{
    float current_x = startx;
    float current_y = starty;
    float current_z = startz;

    float step_x = (endx - startx) / steps;
    float step_y = (endy - starty) / steps;
    float step_z = (endz - startz) / steps;

    float join_angles[3];

    for (int i = 0; i <= steps; i++){
        calculate_ik(current_x, current_y, current_z, join_angles);

        set_pwm_angle(1, join_angles[0], freq);
        set_pwm_angle(2, join_angles[1], freq);
        set_pwm_angle(3, join_angles[2], freq);

        current_x += step_x;
        current_y += step_y;
        current_y += step_z;

        usleep(1000000 / freq);
    }
}
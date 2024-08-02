#include "trajectory.h"
#include "bezier.h"
#include <stdio.h>

void calculate_swing_stance_phase(float startx, float startz, float stride_length,
                                  float swing_height, int leg_type, float *controlx,
                                  float *controlz, float *endx, float *endz, bool is_swing_phase)
{
    if (leg_type == 1) {
        *controlx = startx + (is_swing_phase ? stride_length / 2 : -stride_length / 2);
        *endx = startx + (is_swing_phase ? stride_length : -stride_length);
    } else if (leg_type == -1) {
        *controlx = startx - (is_swing_phase ? stride_length / 2 : -stride_length / 2);
        *endx = startx - (is_swing_phase ? stride_length : -stride_length);
    }

    if (is_swing_phase) {
        *controlz = startz + 2 * swing_height;
        *endz = startz;
    } else {
        *controlz = startz - SWING_PUSH_BACK;
        *endz = startz;
    }
}

void generate_swing_phase(struct bezier2d *curve, float startx, float startz, float stride_length,
                          float swing_height, int leg_type)
{
    float controlx, controlz, endx, endz;

    calculate_swing_stance_phase(startx, startz, stride_length, swing_height, leg_type, &controlx,
                                 &controlz, &endx, &endz, true);
    bezier2d_generate_curve(curve, startx, startz, controlx, controlz, endx, endz);
}

void generate_stance_phase(struct bezier2d *curve, float startx, float startz, float stride_length,
                           int leg_type)
{
    float controlx, controlz, endx, endz;
    calculate_swing_stance_phase(startx, startz, stride_length, 0, leg_type, &controlx, &controlz,
                                 &endx, &endz, false);
    bezier2d_generate_curve(curve, startx, startz, controlx, controlz, endx, endz);
}

void generate_walk_trajectory(struct bezier2d *curve, SpiderLeg *leg, float stride_length,
                              float swing_height, LegPosition leg_positions)
{
    // get current position
    float startx_swing = leg->joints[3][0] - stride_length;
    float startz_swing = leg->joints[3][2];

    // create swing phase
    generate_swing_phase(curve, startx_swing, startz_swing, stride_length, swing_height - 20.0,
                         FRONT);

    // update start position for stance phase
    float startx_stance = startx_swing + stride_length;
    float startz_stance = startz_swing;

    generate_stance_phase(curve, startx_stance, startz_stance, stride_length, FRONT);
}

void generate_walk_back_leg_trajectory(struct bezier2d *curve, SpiderLeg *leg, float stride_length,
                                       float swing_height, LegPosition leg_positions)
{
    // get current position
    float startx_swing = leg->joints[3][0];
    float startz_swing = leg->joints[3][2];

    // create swing phase
    generate_swing_phase(curve, startx_swing, startz_swing, stride_length, swing_height + 20, BACK);

    // update start position for stance phase
    float startx_stance = startx_swing - stride_length;
    float startz_stance = startz_swing;

    generate_stance_phase(curve, startx_stance, startz_stance, stride_length, BACK);
}

void print_trajectory(struct bezier2d *curve, int num_points)
{
    printf("Trajectory Points:\n");
    for (int i = 0; i <= num_points; i++) {
        float t = (float)i / num_points;
        float x, y;
        bezier2d_getPos(curve, t, &x, &y);
        printf("Point %d: (%.2f, %.2f)\n", i, x, y);
    }
}

void print_trajectory_3d(struct bezier3d *curve, int num_points)
{
    printf("Trajectory Points:\n");
    for (int i = 0; i <= num_points; i++) {
        float t = (float)i / num_points;
        float x, y, z;
        bezier3d_getpos(curve, t, &x, &y, &z);
        printf("Point %d: (%.2f, %.2f, %.2f)\n", i, x, y, z);
    }
}

void save_trajectory_points(struct bezier2d *curve, const char *filename, int num_points)
{
    FILE *file = fopen(filename, "w");
    if (file == NULL) {
        printf("Error opening file.\n");
        return;
    }

    for (int i = 0; i <= num_points; i++) {
        float t = (float)i / num_points;
        float x, z;
        bezier2d_getPos(curve, t, &x, &z);
        fprintf(file, "%.2f %.2f\n", x, z);
    }

    fclose(file);
}

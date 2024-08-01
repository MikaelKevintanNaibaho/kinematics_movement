#include "move.h"
#include <unistd.h>

void generate_turn_left_trajectory(struct bezier3d *curve, SpiderLeg *leg, float stride_length,
                                   float swing_height, LegPosition position_leg)
{
    // ambil posisi terkini pada 3d coordinate
    float startx = leg->joints[3][0];
    float starty = leg->joints[3][1];
    float startz = leg->joints[3][2];

    // define control point untuk belok kiri
    float controlx = startx + 2 * startx;
    float controly = starty - stride_length / 2;
    float controlz = startz + 2 * swing_height;

    float endx = startx;
    float endy = starty + stride_length;
    float endz = startz;

    bezier3d_generate_curve(curve, startx, starty, startz, controlx, controly, controlz, endx, endy,
                            endz);
}

void update_leg_trot_gait(struct bezier2d curve[NUM_LEGS], int num_points,
                          SpiderLeg *legs[NUM_LEGS], LegPosition leg_positions[NUM_LEGS])
{
    float desired_duration = DESIRED_TIME;
    float dt = desired_duration / num_points;

    for (int i = 0; i <= num_points; i++) {
        float t = (float)i / num_points;

        // Define phase offsets for trot gait
        float phase_offsets[NUM_LEGS] = { 0.0, 0.5, 0.0, 0.5 }; // Diagonal pairs

        // Calculate positions for each leg based on the phase offsets
        float x[NUM_LEGS], z[NUM_LEGS];
        for (int j = 0; j < NUM_LEGS; j++) {
            float phase_offset = fmod(t + phase_offsets[j], 1.0);
            bezier2d_getPos(&curve[j], phase_offset, &x[j], &z[j]);
        }

        // Update leg positions using inverse kinematics
        for (int j = 0; j < NUM_LEGS; j++) {
            inverse_kinematics(legs[j], (float[]) { x[j], legs[j]->joints[3][1], z[j] },
                               leg_positions[j]);
        }

        usleep((long)(dt * 1e6));
    }
}

void update_leg_left(struct bezier3d curve[NUM_LEGS], int num_points, SpiderLeg *legs[NUM_LEGS],
                     LegPosition leg_positions[NUM_LEGS])
{
    float desired_duration = DESIRED_TIME;
    float dt = desired_duration / num_points;

    // Define the desired gait pattern for each leg (phase offsets)
    float phase_offsets[NUM_LEGS] = { 0.0, 0.25, 0.5, 0.75 }; // Example: Trot gait

    for (int i = 0; i <= num_points; i++) {
        float t = (float)i / num_points;

        // Update positions for each leg based on the gait pattern
        float x[NUM_LEGS], y[NUM_LEGS], z[NUM_LEGS];
        for (int j = 0; j < NUM_LEGS; j++) {
            float phase_offset = fmod(t + phase_offsets[j], 1.0);
            bezier3d_getpos(&curve[j], phase_offset, &x[j], &y[j], &z[j]);
        }

        // Update leg positions using inverse kinematics
        for (int j = 0; j < NUM_LEGS; j++) {
            inverse_kinematics(legs[j], (float[]) { x[j], y[j], z[j] }, leg_positions[j]);
        }

        usleep((long)(dt * 1e6));
    }
}

const char *leg_position_to_string(LegPosition position)
{
    switch (position) {
    case KIRI_DEPAN:
        return "KIRI_DEPAN";
    case KIRI_BELAKANG:
        return "KIRI_BELAKANG";
    case KANAN_BELAKANG:
        return "KANAN_BELAKANG";
    case KANAN_DEPAN:
        return "KANAN_DEPAN";
    default:
        return "Unknown";
    }
}

void stand_position(void)
{
    for (int i = 0; i < NUM_LEGS; i++) {
        printf("standby position for Leg %s (Position %d):\n", legs[i]->name, leg_positions[i]);
        set_angles(legs[i], stance_angles[i]);
        forward_kinematics(legs[i], stance_angles[i], leg_positions[i]);
        printf("----------------------------\n");
    }
}

void move_forward(void)
{
    struct bezier2d curve[NUM_LEGS];
    for (int i = 0; i < NUM_LEGS; i++) {
        bezier2d_init(&curve[i]);
        if (leg_positions[i] == KANAN_BELAKANG || leg_positions[i] == KIRI_BELAKANG) {
            generate_walk_back_leg_trajectory(&curve[i], legs[i], STRIDE_LENGTH, SWING_HEIGHT,
                                              leg_positions[i]);
        } else {
            generate_walk_trajectory(&curve[i], legs[i], STRIDE_LENGTH, SWING_HEIGHT,
                                     leg_positions[i]);
        }
    }

    while (is_program_running) {
        update_leg_trot_gait(curve, NUM_POINTS, legs, leg_positions);
        usleep(100);
    }
}

void move_left_turn(void)
{
    struct bezier3d curve[NUM_LEGS];
    for (int i = 0; i < NUM_LEGS; i++) {
        bezier3d_init(&curve[i]);
        generate_turn_left_trajectory(&curve[i], legs[i], STRIDE_LENGTH, SWING_HEIGHT,
                                      leg_positions[i]);
        print_trajectory_3d(&curve[i], NUM_POINTS);
    }

    while (is_program_running) {
        update_leg_left(curve, NUM_POINTS, legs, leg_positions);
        usleep(100000);
    }
}

#include "move.h"

void bezier2d_generate_straight_back(struct bezier2d *stright_back, float startx, float startz,
                                     float endx, float endy)
{
    bezier2d_addPoint(stright_back, startx, startz);
    bezier2d_addPoint(stright_back, endx, endy);
}

void generate_stright_back_trajectory(struct bezier2d *stright_back, SpiderLeg *leg,
                                      float stride_length)
{
    float startx = leg->joints[3][0];
    float startz = leg->joints[3][2];
    printf("startx : %f", startx);

    float endx = startx - stride_length / 2;
    float endz = startz;

    bezier2d_generate_straight_back(stright_back, startx, startz, endx, endz);
}

void generate_turn_left_trajectory(struct bezier3d *curve, SpiderLeg *leg, float stride_length,
                                   float swing_height, LegPosition position_leg)
{
    // ambil posisi terkini pada 3d coordinate
    float startx = leg->joints[3][0];
    float starty = leg->joints[3][1];
    float startz = leg->joints[3][2];

    // define control point untuk belok kiri
    float controlx = startx +  2 * startx;
    float controly = starty - stride_length / 2;
    float controlz = startz + 2 * swing_height;

    float endx = startx;
    float endy = starty + stride_length;
    float endz = startz;

    bezier3d_generate_curve(curve, startx, starty, startz, controlx, controly, controlz, endx, endy,
                            endz);
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

void print_trajectory_3d(struct bezier3d *curve, int num_points) {
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

void update_leg_wave_gait(struct bezier2d curve[NUM_LEGS], int num_points,
                          SpiderLeg *legs[NUM_LEGS], LegPosition leg_positions[NUM_LEGS])
{
    float desired_duration = DESIRED_TIME;
    float dt = desired_duration / num_points;

    for (int i = 0; i <= num_points; i++) {
        float t = (float)i / num_points;

        // Calculate positions for each leg based on the phase offsets
        float x[NUM_LEGS], z[NUM_LEGS];
        // Declare phase_offsets here so it's accessible throughout the loop
        float phase_offsets[NUM_LEGS];
        float phase_multiplier = 0.5;
        for (int j = 0; j < NUM_LEGS; j++) {
            float phase_offset = t + (float)(j % 2) / (2.0f * NUM_LEGS); // Adjust for tripod stance
            phase_offsets[j] = fmod(phase_offset, 1.0);
            bezier2d_getPos(&curve[j], phase_offsets[j], &x[j], &z[j]);
        }

        for (int j = 0; j < NUM_LEGS; j++) {
            printf("Y value at joints[3][1] for leg %d: %f\n", j, legs[j]->joints[3][1]);
        }

        // Update leg positions using inverse kinematics
        for (int j = 0; j < NUM_LEGS; j++) {
            printf("------------------------------\n");
            inverse_kinematics(legs[j], (float[]) { x[j], legs[j]->joints[3][1], z[j] },
                               leg_positions[j]);
            printf("Leg Position: %s\n", leg_position_to_string(leg_positions[j]));
            usleep(10000);
        }

        usleep((long)(dt * 1e6));
    }
}

void update_leg_trot_gait(struct bezier2d swing_curve[NUM_LEGS], struct bezier2d stance_curve[NUM_LEGS], int num_points,
                          SpiderLeg *legs[NUM_LEGS], LegPosition leg_positions[NUM_LEGS])
{
    float desired_duration = DESIRED_TIME;
    float dt = desired_duration / num_points;

    float t = 0.0;

    while (is_program_running) {
        // Define phase offsets for trot gait
        float phase_offsets[NUM_LEGS] = { 0.0, 0.5, 0.0, 0.5 }; // Diagonal pairs

        // Calculate positions for each leg based on the phase offsets
        float x[NUM_LEGS], z[NUM_LEGS];
        for (int i = 0; i < NUM_LEGS; i++) {
            float phase_offset = fmod(t + phase_offsets[i], 1.0);
            struct bezier2d *curve;
            if (phase_offset < 0.5) {
                // Swing phase
                curve = &swing_curve[i];
                phase_offset *= 2.0; // Normalize to 0-1 range for the swing phase
            } else {
                // Stance phase
                curve = &stance_curve[i];
                phase_offset = (phase_offset - 0.5) * 2.0; // Normalize to 0-1 range for the stance phase
            }
            bezier2d_getPos(curve, phase_offset, &x[i], &z[i]);
        }

        // Update leg positions using inverse kinematics
        for (int i = 0; i < NUM_LEGS; i++) {
            inverse_kinematics(legs[i], (float[]){ x[i], legs[i]->joints[3][1], z[i] }, leg_positions[i]);
        }

        // Increment t for the next iteration
        t += dt;
        if (t >= 1.0) {
            t = 0.0; // Reset t after a complete cycle
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
            inverse_kinematics(legs[j], (float[]){ x[j], y[j], z[j] }, leg_positions[j]);
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
    struct bezier2d swing_curve[NUM_LEGS];
    struct bezier2d stance_curve[NUM_LEGS];

    // Generate swing and stance trajectories for each leg
    for (int i = 0; i < NUM_LEGS; i++) {
        bezier2d_init(&swing_curve[i]);
        bezier2d_init(&stance_curve[i]);
        if (leg_positions[i] == KIRI_DEPAN || leg_positions[i] == KANAN_DEPAN) {
            generate_swing_phase(&swing_curve[i], legs[i]->joints[3][0], legs[i]->joints[3][2],
                                 STRIDE_LENGTH, SWING_HEIGHT, 1);
            generate_stance_phase(&stance_curve[i], legs[i]->joints[3][0], legs[i]->joints[3][2],
                                  STRIDE_LENGTH, 1);
        } else {
            generate_swing_phase(&swing_curve[i], legs[i]->joints[3][0], legs[i]->joints[3][2],
                                 STRIDE_LENGTH, SWING_HEIGHT, -1);
            generate_stance_phase(&stance_curve[i], legs[i]->joints[3][0], legs[i]->joints[3][2],
                                  STRIDE_LENGTH, -1);
        }
    }

    while (is_program_running) {
        update_leg_trot_gait(swing_curve, stance_curve, NUM_POINTS, legs, leg_positions);
        usleep(10000);
    }
}




void move_left_turn(void)
{
    struct bezier3d curve[NUM_LEGS];
    for(int i = 0; i < NUM_LEGS; i++) {
        bezier3d_init(&curve[i]);
        generate_turn_left_trajectory(&curve[i], legs[i], STRIDE_LENGTH, SWING_HEIGHT, leg_positions[i]);
        print_trajectory_3d(&curve[i], NUM_POINTS);
    }
    
    while(is_program_running) {
        update_leg_left(curve, NUM_POINTS, legs, leg_positions);
        usleep(100000);
    }
}

void adjust_leg_positions(float pitch, float roll, SpiderLeg *legs[NUM_LEGS])
{
    for (int i = 0; i < NUM_LEGS; i++) {
        // Adjust leg positions based on pitch
        if (fabs(pitch) > PITCH_THRESHOLD) {
            float adjustment = pitch > 0 ? LEG_ADJUSTMENT_ANGLE : -LEG_ADJUSTMENT_ANGLE;
            legs[i]->theta1 += adjustment;
        }

        // Adjust leg positions based on roll
        if (fabs(roll) > ROLL_THRESHOLD) {
            float adjustment = roll > 0 ? LEG_ADJUSTMENT_ANGLE : -LEG_ADJUSTMENT_ANGLE;
            legs[i]->theta2 += adjustment;
            legs[i]->theta3 -= adjustment; // Adjust opposite leg segment to maintain balance
        }

        // Ensure leg angles are within valid range
        legs[i]->theta1 = normalize_angle(legs[i]->theta1);
        legs[i]->theta2 = normalize_angle(legs[i]->theta2);
        legs[i]->theta3 = normalize_angle(legs[i]->theta3);
    }
}

void self_balance(float roll, float pitch)
{
    while (is_program_running) {

        // Adjust leg positions based on pitch and roll
        adjust_leg_positions(pitch, roll, legs);

        // Update leg positions with adjusted angles
        for (int i = 0; i < NUM_LEGS; i++) {
            set_angles(legs[i], (float[]){ legs[i]->theta1, legs[i]->theta2, legs[i]->theta3 });
        }

        // Add delay before next iteration to control loop frequency
        usleep(100000); // Adjust as needed based on desired loop frequency
    }
}
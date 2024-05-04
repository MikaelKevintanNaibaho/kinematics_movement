#include "move.h"

void generate_walk_trajectory(struct bezier2d *curve, SpiderLeg *leg, float stride_length,
                              float swing_height, LegPosition position_leg)
{
    // get current position
    float startx = leg->joints[3][0];
    float startz = leg->joints[3][2];
    printf("startx = %f, startz %f\n", startx, startz);

    // control points
    float controlx = startx + stride_length / 2;

    printf("controlx = %f \t", controlx);
    float controlz = startz + 2 * swing_height;
    printf("controlz = %f\n", controlz);
    float endx_forward = startx + stride_length;
    float endz_forward = startz;
    // buar bezier curve
    bezier2d_generate_curve(curve, startx, startz, controlx, controlz, endx_forward, endz_forward);

    // Append straight line for moving backward
    bezier2d_addPoint(curve, endx_forward, endz_forward);
    bezier2d_addPoint(curve, startx, startz);
}

void generate_walk_back_leg(struct bezier2d *curve, SpiderLeg *leg, float stride_length,
                            float swing_height, LegPosition leg_position)
{
    float startx = leg->joints[3][0] + stride_length;
    float startz = leg->joints[3][2];
    printf("startx = %f, startz %f\n", startx, startz);

    float controlx = startx - stride_length / 2;

    printf("controlx = %f \t", controlx);
    float controlz = startz + 2 * swing_height;
    printf("controlz = %f\n", controlz);
    float endx_forward = startx - stride_length;
    float endz_forward = startz;
    // buar bezier curve
    bezier2d_generate_curve(curve, startx, startz, controlx, controlz, endx_forward, endz_forward);

    // Append straight line for moving backward
    bezier2d_addPoint(curve, endx_forward, endz_forward);
    bezier2d_addPoint(curve, startx, startz);
}

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
void update_leg_position_with_velocity(struct bezier2d *curve, int number_points, SpiderLeg *leg,
                                       LegPosition position_leg)
{
    printf("updating leg position with fixed delay\n");

    float prev_x, prev_z;
    bezier2d_getPos(curve, 0, &prev_x, &prev_z); // get poin pertama

    for (int i = 0; i <= number_points; i++) {
        float t = (float)i / number_points;
        float x, z;
        bezier2d_getPos(curve, t, &x, &z);

        prev_x = x;
        prev_z = z;
    }

    // hitung velocity base on total distance dan disired duration
    float desired_duration = DESIRED_TIME;

    // hitung interval antara dua titik
    float dt = desired_duration / number_points;

    // lakukan untuk setiap titik dan control velocity
    // Iterate over points and control velocity
    for (int i = 0; i <= number_points; i++) {
        float t = (float)i / number_points;
        float x, z;
        bezier2d_getPos(curve, t, &x, &z);

        // Print position of current point
        printf("Point %d: (%.2f, %.2f)\n", i, x, z);

        // Calculate target position for inverse kinematics
        float y = leg->joints[3][1];
        float target_positions[3] = { x, y, z };

        // Update leg position using inverse kinematics
        inverse_kinematics(leg, target_positions, position_leg);

        // Introduce delay based on desired velocity
        long delay = (long)(dt * 1e6);
        usleep(delay);
    }
}

void update_leg_wave_gait(struct bezier2d curve[NUM_LEGS], int num_points,
                          SpiderLeg *legs[NUM_LEGS], LegPosition leg_positions[NUM_LEGS])
{
    float desired_duration = DESIRED_TIME;
    float dt = desired_duration / num_points;

    for (int i = 0; i <= num_points; i++) {
        float t = (float)i / num_points;

        // Calculate phase offsets for each leg
        float phase_offsets[NUM_LEGS];
        for (int j = 0; j < NUM_LEGS; j++) {
            phase_offsets[j] =
                fmod(t + j * (1.0 / NUM_LEGS), 1.0); // Adjust phase offsets for each leg
        }

        // Calculate positions for each leg based on the phase offsets
        float x[NUM_LEGS], z[NUM_LEGS];
        for (int j = 0; j < NUM_LEGS; j++) {
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
        }

        usleep((long)(dt * 1e6));
    }
}

void update_leg_trot_gait(struct bezier2d curve[NUM_LEGS], int num_points,
                           SpiderLeg *legs[NUM_LEGS], LegPosition leg_positions[NUM_LEGS])
{
    float desired_duration = DESIRED_TIME;
    float dt = desired_duration / num_points;

    for (int i = 0; i <= num_points; i++) {
        float t = (float)i / num_points;

        // Calculate phase offsets for each leg in a crawl gait
        float phase_offsets[NUM_LEGS];
        for (int j = 0; j < NUM_LEGS; j++) {
            // Adjust phase offsets for diagonal leg movement
            phase_offsets[j] = fmod(t + (j % 2 == 0 ? 0.25 : 0.75), 1.0);
        }

        // Calculate positions for each leg based on the phase offsets
        float x[NUM_LEGS], z[NUM_LEGS];
        for (int j = 0; j < NUM_LEGS; j++) {
            bezier2d_getPos(&curve[j], phase_offsets[j], &x[j], &z[j]);
        }

        // Update leg positions using inverse kinematics
        for (int j = 0; j < NUM_LEGS; j++) {
            printf("------------------------------\n");
            // For crawl gait, adjust leg positions to create diagonal movement
            // float z_offset = (j % 2 == 0) ? LEG_HEIGHT_OFFSET : -LEG_HEIGHT_OFFSET;

            //   if (j == 1 || j == 4) {
            //     z_offset *= -1;
            // }
            inverse_kinematics(legs[j], (float[]) { x[j], legs[j]->joints[3][1], z[j] },
                               leg_positions[j]);
            printf("Leg Position: %s\n", leg_position_to_string(leg_positions[j]));
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
            generate_walk_back_leg(&curve[i], legs[i], STRIDE_LENGTH, SWING_HEIGTH,
                                   leg_positions[i]);
        } else {
            generate_walk_trajectory(&curve[i], legs[i], STRIDE_LENGTH, SWING_HEIGTH,
                                     leg_positions[i]);
        }
        print_trajectory(&curve[i], 30);
    }

    while (1) {
        update_leg_trot_gait(curve, NUM_POINTS, legs, leg_positions);
    }
}
#include "ik.h"

float degrees(float rad)
{
    return rad * (180.0 / M_PI);
}

float radians(float deg)
{
    return deg * (M_PI / 180.0);
}

float normalize_angle(float angle)
{
    angle = fmodf(angle,
                  360.0); // Ensure angle is within the range of -360.0 to 360.0

    // Convert negative angles to their corresponding positive angles within the
    // same position
    if (angle < 0)
        angle += 360.0;

    // Ensure angle is within the range of 0.0 to 180.0
    if (angle > 180.0)
        angle = 360.0 - angle;

    return angle;
}

float *get_target(SpiderLeg *leg)
{
    return leg->joints[3];
}

void set_angles(SpiderLeg *leg, float angles[3])
{
    leg->theta1 = normalize_angle(angles[0]);
    leg->theta2 = normalize_angle(angles[1]);
    leg->theta3 = normalize_angle(angles[2]);

    for (int i = 0; i < 3; i++) {
        set_pwm_angle(leg->servo_channles[i], (int)angles[i], PWM_FREQ);
        printf("theta%d: %.2f degrees\n", i + 1, angles[i]);
    }
}

// Helper function to check if two sets of angles are approximately equal
int angles_equal(float angles1[3], float angles2[3])
{
    for (int i = 0; i < 3; ++i) {
        if (fabs(angles1[i] - angles2[i]) > 0.01) {
            return 0;
        }
    }
    return 1;
}

void move_to_angle(SpiderLeg *leg, float target_angles[3], int speed)
{
    float current_angles[3] = { leg->theta1, leg->theta2, leg->theta3 };

    // hitung max change dalam angle per step berdasarkan speed
    float delta_theta = DELTA_THETA_MAX * (float)speed / 1.0;

    float delta_directions[3];
    for (int i = 0; i < 3; i++) {
        delta_directions[i] = (target_angles[i] > current_angles[i]) ? 1.0 : -1.0;
    }

    // adjust angle gradually toward the target
    while (!angles_equal(current_angles, target_angles)) {
        for (int i = 0; i < 3; i++) {
            // hitung next angle berdasarkan current_angles dan delta_theta
            float next_angle = current_angles[i] + delta_theta * delta_directions[i];

            // make sure the next angle nggk overshoot dari target angles
            if ((delta_directions[i] > 0 && next_angle > target_angles[i])
                || (delta_directions[i] < 0 && next_angle < target_angles[i])) {
                next_angle = target_angles[i];
            }

            current_angles[i] = next_angle;
        }

        // set servo angle dan kasih delay
        set_angles(leg, current_angles);
        usleep(DELAY_US);
    }
}

void init_DH_params(DHParameters *params, float alpha, float a, float d, float theta)
{
    params->alpha = alpha;
    params->a = a;
    params->d = d;
    params->theta = theta;
}

void create_DH_matrix(const DHParameters *params, gsl_matrix *matrix)
{
    float alpha = params->alpha;
    float theta = params->theta;

    // Fill the DH matrix
    gsl_matrix_set(matrix, 0, 0, cos(theta));
    gsl_matrix_set(matrix, 0, 1, -sin(theta) * cos(alpha));
    gsl_matrix_set(matrix, 0, 2, sin(theta) * sin(alpha));
    gsl_matrix_set(matrix, 0, 3, params->a * cos(theta));

    gsl_matrix_set(matrix, 1, 0, sin(theta));
    gsl_matrix_set(matrix, 1, 1, cos(theta) * cos(alpha));
    gsl_matrix_set(matrix, 1, 2, -cos(theta) * sin(alpha));
    gsl_matrix_set(matrix, 1, 3, params->a * sin(theta));

    gsl_matrix_set(matrix, 2, 0, 0.0);
    gsl_matrix_set(matrix, 2, 1, sin(alpha));
    gsl_matrix_set(matrix, 2, 2, cos(alpha));
    gsl_matrix_set(matrix, 2, 3, params->d);

    gsl_matrix_set(matrix, 3, 0, 0.0);
    gsl_matrix_set(matrix, 3, 1, 0.0);
    gsl_matrix_set(matrix, 3, 2, 0.0);
    gsl_matrix_set(matrix, 3, 3, 1.0);
}

void print_DH_matrix(const DHMatrix *matrix)
{
    printf("DH Matrix:\n");
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            printf("%f\t", matrix->matrix[i][j]);
        }
        printf("\n");
    }
}

void multiply_DH_matrices(const DHMatrix *matrix1, const DHMatrix *matrix2, DHMatrix *result)
{
    int i, j, k;
    for (i = 0; i < 4; i++) {
        for (j = 0; j < 4; j++) {
            result->matrix[i][j] = 0;
            for (k = 0; k < 4; k++) {
                result->matrix[i][j] += matrix1->matrix[i][k] * matrix2->matrix[k][j];
            }
        }
    }
}

void calculate_DH_transformation(const DHParameters *params_array, int num_links,
                                 gsl_matrix *result)
{
    // Identity matrix
    gsl_matrix *identityMatrix = gsl_matrix_alloc(4, 4);
    gsl_matrix_set_identity(identityMatrix);

    // Iterate through each link and calculate intermediate matrices
    for (int i = 0; i < num_links; i++) {
        gsl_matrix *linkMatrix = gsl_matrix_alloc(4, 4);
        create_DH_matrix(&params_array[i], linkMatrix);

        // Multiply identityMatrix and linkMatrix
        gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, identityMatrix, linkMatrix, 0.0, result);

        // Update the identityMatrix with the multiplied matrix
        gsl_matrix_memcpy(identityMatrix, result);

        // Free the linkMatrix since it's not needed anymore
        gsl_matrix_free(linkMatrix);
    }

    // Free the identityMatrix
    gsl_matrix_free(identityMatrix);
}

void forward_kinematics(SpiderLeg *leg, float angles[3], LegPosition position_leg)
{
    // Convert to radians
    float theta1 = radians(angles[0]);
    float theta2 =
        radians(angles[1]) - radians(90); // -90 because of angle offset of mounting servo
    float theta3 =
        -radians(angles[2]) + radians(90); // -90 because of angle offset of mounting_servo

    float zero_offset = 0.0;
    switch (position_leg) {
    case KANAN_DEPAN:
        zero_offset = 0.0;
        break;
    case KIRI_DEPAN:
        zero_offset = 90.0;
        break;
    case KIRI_BELAKANG:
        zero_offset = 180.0;
        break;
    case KANAN_BELAKANG:
        zero_offset = 90.0;
    default:
        break;
    }
    theta1 += radians(zero_offset);

    DHParameters params_array[NUM_LINKS];
    init_DH_params(&params_array[0], radians(90.0), COXA_LENGTH, 0.0, (theta1 + radians(90.0)));
    init_DH_params(&params_array[1], radians(0.0), FEMUR_LENGTH, 0.0, theta2);
    init_DH_params(&params_array[2], radians(-90.0), TIBIA_LENGTH, 0.0, (theta3 - radians(90.0)));
    init_DH_params(&params_array[3], radians(90.0), 0.0, 0.0, radians(-90.0));

    gsl_matrix *trans_matrix = gsl_matrix_alloc(4, 4);
    calculate_DH_transformation(params_array, NUM_LINKS, trans_matrix);

    float x = fabs(gsl_matrix_get(trans_matrix, 0, 3));
    if (x < 0) {
        x = 0;
    }

    if (position_leg == KANAN_BELAKANG || position_leg == KIRI_BELAKANG) {
        x = -x;
    }
    float y = gsl_matrix_get(trans_matrix, 1, 3);
    float z = gsl_matrix_get(trans_matrix, 2, 3);

    float position[3] = { x, y, z };

    // Update leg joints end-effector
    for (int i = 0; i < 3; i++) {
        leg->joints[3][i] = position[i];
    }

    printf("end-effector position: x = %.2f, y = %.2f, z = %.2f\n", leg->joints[3][0],
           leg->joints[3][1], leg->joints[3][2]);

    gsl_matrix_free(trans_matrix);
}

void inverse_kinematics(SpiderLeg *leg, float target_positions[3], LegPosition position_leg)
{
    // float x;
    // if((target_positions[0]) < 0){
    //    if (target_positions[0] + leg->joints[3][0] < 0){
    //         x = leg->joints[3][0];
    //         printf("not posible\n");
    //     }else{
    //         x = target_positions[0] + leg->joints[3][0];
    //     }

    // } else if(target_positions[0] > 0){
    //     x = target_positions[0] + leg->joints[3][0];
    // } else {
    //     x = leg->joints[3][0];
    // }

    // printf("x = %.2f\n", x);

    // float y;
    // if (target_positions[1] < 0){
    //     if (target_positions[1] + leg->joints[3][1] < 0){
    //         y = target_positions[1] + leg->joints[3][1];
    //         y = fabs(y);
    //     } else {
    //         y = target_positions[1] + leg->joints[3][1];
    //     }
    // } else if (target_positions[1] > 0){
    //     y = target_positions[1] + leg->joints[3][1];
    // } else {
    //     y = leg->joints[3][1];
    // }

    // printf("y = %.2f\n", y);

    // float z;
    // if (target_positions[2] < 0){
    //     z = target_positions[2] + leg->joints[3][2];
    // } else if(target_positions[2] > 0 ){
    //     z = target_positions[2] - leg->joints[3][2];
    // } else {
    //     z = leg->joints[3][2];
    // }
    // printf("z = %.2f\n", z);
    // z = fabs(z);
    float x = target_positions[0];
    float y = target_positions[1];
    float z = target_positions[2];

    adjust_coordinate(x, y, z, position_leg, &x, &y, &z);
    // angle antara coxa dengan horizontal plane
    float theta1 = atan2(x, y);

    float P = sqrt(pow(x, 2) + powf(y, 2)) - COXA_LENGTH;

    float G = sqrt(pow(z, 2) + pow(P, 2));

    float alpha = atan2(z, P);

    float gamma_cos =
        (pow(FEMUR_LENGTH, 2) + pow(G, 2) - pow(TIBIA_LENGTH, 2)) / (2 * FEMUR_LENGTH * G);
    float gamma = acos(gamma_cos);

    float beta_cos = (pow(FEMUR_LENGTH, 2) + pow(TIBIA_LENGTH, 2) - pow(G, 2))
        / (2 * FEMUR_LENGTH * TIBIA_LENGTH);
    float beta = acos(beta_cos);

    float theta2 = M_PI / 2 + (gamma - fabs(alpha));

    float theta3 = M_PI - beta;

    // Convert angles to degrees
    theta1 = degrees(theta1);
    theta2 = degrees(theta2);
    theta3 = degrees(theta3);

    // if (theta1 > 90) {
    //     theta1 = 180.0 - theta1;
    // }

    // if (position_leg == KANAN_BELAKANG || position_leg == KIRI_BELAKANG) {
    //     theta1 += 90.0;
    // }

    // Ensure angles are within the valid range
    // theta1 = normalize_angle(theta1);
    // theta2 = normalize_angle(theta2);
    // theta3 = normalize_angle(theta3);

    float angles[3] = { theta1, theta2, theta3 };
    set_angles(leg, angles);
    forward_kinematics(leg, angles, position_leg);
    // printf("theta1 = %.2f, theta2 = %.2f, theta3 = %.2f\n", theta1, theta2, theta3);
}

void initialize_leg(SpiderLeg *leg, const char *name, int servo_ch1, int servo_ch2, int servo_ch3)
{
    strcpy(leg->name, name);
    leg->servo_channles[0] = servo_ch1;
    leg->servo_channles[1] = servo_ch2;
    leg->servo_channles[2] = servo_ch3;
}

void initialize_all_legs(SpiderLeg *legs[NUM_LEGS])
{
    initialize_leg(legs[0], "KIRI_DEPAN", SERVO_CHANNEL_1, SERVO_CHANNEL_2, SERVO_CHANNEL_3);
    initialize_leg(legs[1], "KIRI_BELAKANG", SERVO_CHANNEL_4, SERVO_CHANNEL_5, SERVO_CHANNEL_6);
    initialize_leg(legs[2], "KANAN_BELAKANG", SERVO_CHANNEL_7, SERVO_CHANNEL_8, SERVO_CHANNEL_9);
    initialize_leg(legs[3], "KANAN_DEPAN", SERVO_CHANNEL_10, SERVO_CHANNEL_11, SERVO_CHANNEL_12);
}

void adjust_angle(float theta1, float theta2, float theta3, LegPosition position, float *adj_theta1,
                  float *adj_theta2, float *adj_theta3)
{
    switch (position) {
    case KANAN_DEPAN:
        *adj_theta1 = theta1;
        *adj_theta2 = theta2;
        *adj_theta3 = theta3;
        break;
    case KANAN_BELAKANG:
        *adj_theta1 = -theta1;
        *adj_theta2 = theta2;
        *adj_theta3 = theta3;
        break;
    case KIRI_BELAKANG:
        *adj_theta1 = theta1 + M_PI;
        *adj_theta2 = theta2;
        *adj_theta3 = theta3;
        break;
    case KIRI_DEPAN:
        *adj_theta1 = M_PI / 2 - (theta1 - M_PI / 2);
        *adj_theta2 = theta2;
        *adj_theta3 = theta3;
        break;
    default:
        *adj_theta1 = theta1;
        *adj_theta2 = theta2;
        *adj_theta3 = theta3;
        break;
    }
}

void adjust_coordinate(float x, float y, float z, LegPosition position, float *adj_x, float *adj_y,
                       float *adj_z)
{
    switch (position) {
    case KANAN_DEPAN:
        *adj_x = x;
        *adj_y = y;
        *adj_z = z;
        break;
    case KANAN_BELAKANG:
        *adj_x = -y;
        *adj_y = x;
        *adj_z = z;
        break;
    case KIRI_BELAKANG:
        *adj_x = -x;
        *adj_y = -y;
        *adj_z = z;
        break;
    case KIRI_DEPAN:
        *adj_x = -y;
        *adj_y = x;
        *adj_z = z;
        break;
    default:
        // Default case to handle any other unexpected leg positions
        *adj_x = 0.0;
        *adj_y = 0.0;
        *adj_z = 0.0;
        break;
    }
}

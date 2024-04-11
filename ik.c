#include "ik.h"

float degrees(float rad) {
    return rad * (180.0 / M_PI);
}

float radians(float deg) {
    return deg * (M_PI / 180.0);
}

float normalize_angle(float angle) {
    angle = fmodf(angle, 360.0);  // Ensure angle is within the range of -360.0 to 360.0
    
    // Convert negative angles to their corresponding positive angles within the same position
    if (angle < 0) angle += 360.0;

    // Ensure angle is within the range of 0.0 to 180.0
    if (angle > 180.0) angle = 360.0 - angle;

    return angle;
}

float *get_target(SpiderLeg *leg) {
    return leg->joints[3];
}


void set_angles(SpiderLeg *leg, float angles[3]) {
    leg->theta1 = normalize_angle(angles[0]);
    leg->theta2 = normalize_angle(angles[1]);
    leg->theta3 = normalize_angle(angles[2]);

    set_pwm_angle(SERVO_CHANNEL_1, (int)leg->theta1, PWM_FREQ, SERVO_VELOCTY);
    set_pwm_angle(SERVO_CHANNEL_2, (int)leg->theta2,PWM_FREQ, SERVO_VELOCTY);
    set_pwm_angle(SERVO_CHANNEL_3, (int)leg->theta3, PWM_FREQ, SERVO_VELOCTY);

    printf("Theta1: %.4f degrees\n", leg->theta1);
    printf("Theta2: %.4f degrees\n", leg->theta2);
    printf("Theta3: %.4f degrees\n", leg->theta3);
}


IK_ErrorCode inverse_kinematics(SpiderLeg *leg, float target[3])
{
    float x = target[0];
    float y = target[1];
    float z = target[2];

    float initial_x = 100.0;
    float initial_y = 100.0;
    float z_offset;
    if(z > 0 || z == 0){
        z_offset = 100 + (z);
    } else {
        z_offset = 100 - z;
    }

    float theta1;
    if (y == initial_y){
        float distance = sqrtf(x*x + y*y);
        theta1 = (x >= initial_x) ? acosf(x / distance) : -acosf(x / distance);
    } else {
        theta1 = atan2f(x, y);
    }

    if (x == initial_x){
        float distance = sqrtf(x*x + y*y);
        theta1 = (y >= initial_y) ? acosf(y / distance) : -acosf(y / distance);
    }
    float L1 = sqrtf(powf(x, 2) + powf(y, 2));

    float L = sqrtf(powf(z_offset, 2) + powf(L1 - COXA_LENGTH, 2));

    float alpha1;
    if (z_offset >= 0) {
        alpha1 = acosf(z_offset / L);
    } else {
        alpha1 = -acosf(-z_offset / L);
    }
    
    float alpha2_cos = (powf(TIBIA_LENGTH, 2) - powf(FEMUR_LENGTH, 2) - powf(L, 2)) / (-2 * FEMUR_LENGTH * L);
    float alpha2 = acosf(alpha2_cos);

    float theta2 = alpha1 + alpha2;

    float alpha3_cos = ((powf(L, 2) - powf(TIBIA_LENGTH, 2) - powf(FEMUR_LENGTH, 2))) / (-2 * TIBIA_LENGTH * FEMUR_LENGTH);
    float alpha3 = acosf(alpha3_cos);

    float theta3 = M_PI - alpha3;
    if (theta3 >= radians(145)){
         theta3 = radians(145);
         printf("melewati batas fisik tibia kaki. Exiting...\n");
         return IK_ERROR_LIMIT_REACHED;
    }
    float angles[3] = {degrees(theta1), degrees(theta2), degrees(theta3)};
    set_angles(leg, angles);

    printf("x = %.2f, y = %.2f, z = %.2f\n", x, y, z_offset);

    return IK_SUCCESS;
}

void move(SpiderLeg *leg, float target[3]) {

    // Calculate the target position relative to the leg's current position
    float new_position[3];
    new_position[0] =  target[0] + leg->joints[3][0];  // Update x-coordinate relative to current position
    new_position[1] = target[1] + leg->joints[3][1];   // Update y-coordinate relative to current position
    new_position[2] = target[2] + leg->joints[3][2];   // Update z-coordinate relative to current position
    
    // Update the leg with inverse kinematics to reach the new position
    inverse_kinematics(leg, new_position);
}

void handle_error(IK_ErrorCode error_code)
{
    switch (error_code) {
    case IK_ERROR_LIMIT_REACHED:
        printf("Inverse kinematics failed: Physical limit reached.\n");
        // Additional error handling logic...
        break;
    case IK_ERROR_INVALID_INPUT:
        printf("Inverse kinematics failed: Invalid input.\n");
        // Additional error handling logic...
        break;
    // Handle other error codes as needed
    default:
        printf("Inverse kinematics failed: Unknown error.\n");
        // Additional error handling logic...
        break;
    }
}


void init_DH_params(DHParameters *params, float alpha, float a, float d, float theta)
{
    params->alpha = alpha;
    params->a = a;
    params->d = d;
    params->theta = theta;
}
void create_DH_matrix(const DHParameters *params, DHMatrix *matrix)
{
    float alpha = radians(params->alpha);
    float theta = params->theta;

    //fill the DH matrix
    matrix->matrix[0][0] = cos(theta);
    matrix->matrix[0][1] = -sin(theta) * cos(alpha);
    matrix->matrix[0][2] = sin(theta) * sin(alpha);
    matrix->matrix[0][3] = params->a * cos(theta);

    matrix->matrix[1][0] = sin(theta);
    matrix->matrix[1][1] = cos(theta) * cos(alpha);
    matrix->matrix[1][2] = -cos(theta) * sin(alpha);
    matrix->matrix[1][3] = params->a * sin(theta);

    matrix->matrix[2][0] = 0.0;
    matrix->matrix[2][1] = sin(alpha);
    matrix->matrix[2][2] = cos(alpha);
    matrix->matrix[2][3] = params->d;

    matrix->matrix[3][0] = 0.0;
    matrix->matrix[3][1] = 0.0;
    matrix->matrix[3][2] = 0.0;
    matrix->matrix[3][3] = 1.0;
}

void print_DH_matrix(const DHMatrix *matrix) {
    printf("DH Matrix:\n");
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            printf("%f\t", matrix->matrix[i][j]);
        }
        printf("\n");
    }
}

void multiply_DH_matrices(const DHMatrix *matrix1, const DHMatrix *matrix2, DHMatrix *result) {
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

void calculate_DH_transformation(const DHParameters *params_array, int num_links, DHMatrix *result)
{
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            if (i == j) {
                result->matrix[i][j] = 1.0;
            } else {
                result->matrix[i][j] = 0.0;
            }
        }
    }

    // Create DH matrices for each link and multiply them together
    DHMatrix tempMatrix;
    for (int i = 0; i < num_links; i++) {
        DHMatrix linkMatrix;
        create_DH_matrix(&params_array[i], &linkMatrix);
        multiply_DH_matrices(result, &linkMatrix, &tempMatrix);
        *result = tempMatrix; // Update result with the multiplied matrix
    }
}

void forward_kinematics(SpiderLeg *leg, float angles[3])
{
    //convert ke radian
    float theta1 = radians(angles[0]);
    float theta2 = radians(angles[1]);
    float theta3 = radians(angles[2]);

    DHMatrix trans_matrix;
    DHParameters params_array[NUM_LINKS];
    init_DH_params(&params_array[0], radians(90.0), COXA_LENGTH, 0.0, (theta1 + radians(90.0)));
    init_DH_params(&params_array[1], radians(0.0), FEMUR_LENGTH, 0.0, theta2);
    init_DH_params(&params_array[2], radians(-90.0), TIBIA_LENGTH, 0.0, (theta3 - radians(90.0)));
    init_DH_params(&params_array[3], radians(90.0), 0.0, 0.0, radians(-90.0));

    calculate_DH_transformation(params_array, NUM_LINKS, &trans_matrix);

    float x = trans_matrix.matrix[0][3];
    float y = trans_matrix.matrix[1][3];
    float z = trans_matrix.matrix[2][3];

    float position[3] = {x, y, z};

    //update leg joints end-effector
    for (int i = 0; i < 3; i++){
        leg->joints[3][i] = position[i];
    }
}


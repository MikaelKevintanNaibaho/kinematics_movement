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

    set_pwm_angle(SERVO_CHANNEL_1, (int)leg->theta1, PWM_FREQ);
    set_pwm_angle(SERVO_CHANNEL_2, (int)leg->theta2,PWM_FREQ );
    set_pwm_angle(SERVO_CHANNEL_3, (int)leg->theta3, PWM_FREQ);

    printf("Theta1: %.4f degrees\n", leg->theta1);
    printf("Theta2: %.4f degrees\n", leg->theta2);
    printf("Theta3: %.4f degrees\n", leg->theta3);
}

void move_to_angle(SpiderLeg *leg, float target_angles[3], float velocity){
    float increment = velocity / 100.0;
    while (leg->theta1 < normalize_angle(target_angles[0]) && leg->theta2 < target_angles[1] && leg->theta3 < target_angles[3]){
        int angle1 = (int)(leg->theta1 + increment);
        int angle2 = (int)(leg->theta2 + increment);
        int angle3 = (int)(leg->theta3 + increment);
        set_pwm_angle(SERVO_CHANNEL_1, angle1, PWM_FREQ);
        set_pwm_angle(SERVO_CHANNEL_2, angle2, PWM_FREQ);
        set_pwm_angle(SERVO_CHANNEL_3, angle2, PWM_FREQ);
        leg->theta1 = angle1;
        leg->theta2 = angle2;
        leg->theta3 = angle3;
        printf("Theta1: %.4f degrees\n", leg->theta1);
        printf("Theta2: %.4f degrees\n", leg->theta2);
        printf("Theta3: %.4f degrees\n", leg->theta3);
        usleep(10000);

    }
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
    float alpha = params->alpha;
    float theta = params->theta;
    printf("alpha = %.2f\n", alpha);
    printf("theta = %.2f\n", theta);
    printf("cos_theta = %.2f\n", cos(theta));

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
    DHMatrix tempMatrix;
    // Identity matrix
    DHMatrix identityMatrix;
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            if (i == j) {
                identityMatrix.matrix[i][j] = 1.0;
            } else {
                identityMatrix.matrix[i][j] = 0.0;
            }
        }
    }
    print_DH_matrix(&identityMatrix);

    // Iterate through each link and calculate intermediate matrices
    for (int i = 0; i < num_links; i++) {
        DHMatrix linkMatrix;
        create_DH_matrix(&params_array[i], &linkMatrix);
        multiply_DH_matrices(&identityMatrix, &linkMatrix, &tempMatrix);

        // Print the intermediate matrix
        printf("Intermediate Matrix for Link %d:\n", i + 1);
        print_DH_matrix(&tempMatrix);
        printf("\n");

        // Update the identity matrix with the multiplied matrix
        identityMatrix = tempMatrix;
    }

    // Copy the final result to the output matrix
    *result = identityMatrix;
}


void forward_kinematics(SpiderLeg *leg, float angles[3])
{
    //convert ke radian
    float theta1 = radians(angles[0]);
    float theta2 = radians(angles[1]) - radians(90); // -90 becasue angle offset of mounting servo
    float theta3 = -radians(angles[2]) + radians(90); // -90 because angle offset of mounting_servo

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


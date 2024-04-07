#include "ik.h"
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include "pwm_servo.h"


// Define a small epsilon value for floating-point comparisons
#define EPSILON 1e-3
#include "ik.h"

const float leg_zero_offset[3] = {100 , 0 , -120};


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

void forward_kinematics(SpiderLeg *leg, float target[3]) {


    float sin_alpha = sinf((leg->mounted_angle));
    float cos_alpha = cosf(leg->mounted_angle);

    float x_coordiante = cos_alpha * target[0] + sin_alpha * target[1];
    x_coordiante += leg_zero_offset[0];

    float y_coordinate = sin_alpha * target[0] - cos_alpha * target[1];
    y_coordinate += leg_zero_offset[1];

    float z_coordinates = target[2] + leg_zero_offset[2];

    leg->joints[3][0] = x_coordiante;
    leg->joints[3][1] = y_coordinate;
    leg->joints[3][2] = z_coordinates;
}

float *get_target(SpiderLeg *leg) {
    return leg->joints[3];
}

void inverse_kinematics(SpiderLeg *leg, float *target) {

    forward_kinematics(leg, target);

    float x = leg->joints[3][0];
    float y = leg->joints[3][1];
    float z = leg->joints[3][2];
    printf ("x = %.2f\n", x);
    printf ("y = %.2f\n", y);
    printf ("z = %.2f\n", z);

    float no_coxa = sqrtf(powf(x, 2) + powf(y, 2) - COXA_LENGTH);
    float servo3_tip_distance = sqrtf(powf(no_coxa, 2) + powf(z, 2));

    float theta1 = atan2f(y, x);

    float angle_right_side_triangle = degrees(atan2f(z, no_coxa));
    float angle_unequal_triangle_rad = acosf((powf(TIBIA_LENGTH, 2) - powf(servo3_tip_distance, 2) - powf(FEMUR_LENGTH, 2)) / (-2 * servo3_tip_distance * FEMUR_LENGTH));
    float angle_unequal_triangel = degrees(angle_unequal_triangle_rad);

    float angle_unequal_triangle2_rad = acosf((powf(servo3_tip_distance, 2) - powf(TIBIA_LENGTH, 2) - powf(FEMUR_LENGTH, 2)) / (-2 * TIBIA_LENGTH * FEMUR_LENGTH));
    float angle_unequal_triangle2 = degrees(angle_unequal_triangle2_rad);

    float theta2 = angle_right_side_triangle + angle_unequal_triangel;

    float theta3 = 180 - angle_unequal_triangle2;
    float angles[3] = {degrees(theta1), degrees(theta2), degrees(theta3)};

    set_angles(leg, angles);

}







// Function to move a single leg of the hexapod robot forward by distance units
void move_forward(SpiderLeg *leg, float target[3]) {

    // Calculate the new target position after moving forward
    float new_position[3];
    new_position[0] =  target[0];  // Update x-coordinate
    new_position[1] = target[1];
    new_position[2] = target[2];
    
    // Update the leg with inverse kinematics to reach the new position
    inverse_kinematics(leg, new_position);
}


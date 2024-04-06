#include "ik.h"
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include "pwm_servo.h"


// Define a small epsilon value for floating-point comparisons
#define EPSILON 1e-3

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

void forward_kinematics(SpiderLeg *leg) {
    // Convert angles to radians
    float theta1 = radians(leg->theta1);
    float theta2 = radians(leg->theta2);
    float theta3 = radians(leg->theta3);

    // Calculate the coordinates of joint 1
    float x1 = leg->COXA * cosf(theta1);
    float y1 = leg->COXA * sinf(theta1);
    float z1 = 0.0;  // Assuming joint 1 is at the base level

    // Calculate the coordinates of joint 2 relative to joint 1
    float x2 = x1 + leg->FEMUR * cosf(theta2) * cosf(theta1);
    float y2 = y1 + leg->FEMUR * cosf(theta2) * sinf(theta1);
    float z2 = z1 + leg->FEMUR * sinf(theta2);

    // Calculate the coordinates of joint 3 relative to joint 2
    float x3 = x2 + leg->TIBIA * cosf(theta3 + theta2) * cosf(theta1);
    float y3 = y2 + leg->TIBIA * cosf(theta3 + theta2) * sinf(theta1);
    float z3 = z2 + leg->TIBIA * sinf(theta3 + theta2);

    // Update the leg's joint positions
    leg->joints[0][0] = 0.0;  // Base joint
    leg->joints[0][1] = 0.0;
    leg->joints[0][2] = 0.0;

    leg->joints[1][0] = x1;
    leg->joints[1][1] = y1;
    leg->joints[1][2] = z1;

    leg->joints[2][0] = x2;
    leg->joints[2][1] = y2;
    leg->joints[2][2] = z2;

    leg->joints[3][0] = x3;
    leg->joints[3][1] = y3;
    leg->joints[3][2] = z3;

    // Print the calculated joint positions
    printf("Forward kinematics of the leg with initial joint angles:\n");
    printf("Joint 1: (%.2f, %.2f, %.2f)\n", leg->joints[0][0], leg->joints[0][1], leg->joints[0][2]);
    printf("Joint 2: (%.2f, %.2f, %.2f)\n", leg->joints[1][0], leg->joints[1][1], leg->joints[1][2]);
    printf("Joint 3: (%.2f, %.2f, %.2f)\n", leg->joints[2][0], leg->joints[2][1], leg->joints[2][2]);
    printf("End Effector: (%.2f, %.2f, %.2f)\n", leg->joints[3][0], leg->joints[3][1], leg->joints[3][2]);
}

float *get_target(SpiderLeg *leg) {
    return leg->joints[3];
}

void inverse_kinematics(SpiderLeg *leg, float *target) {
    float x = target[0];
    float y = 50+ target[1];
    float z = -50 + target[2];

    printf ("x = %.2f\n", x);
    printf ("y = %.2f\n", y);
    printf ("z = %.2f\n", z);

    float theta1 = atan2f(y , x);

    float ya = leg->COXA * sinf(theta1);
    float xa = leg->COXA * cosf(theta1);

    float yb = y - ya;
    float xb = x - xa;

    float h = sqrtf(powf(xb, 2) + powf(yb, 2));

    float l = sqrtf(powf(h, 2) + powf(z, 2));

    float theta3_raw = acosf((powf(leg->FEMUR, 2) + powf(leg->TIBIA, 2) - powf(l, 2)) / (2 * leg->FEMUR * leg->TIBIA));
    float theta3  =  theta3_raw ;

    float phi1 = acosf((powf(leg->FEMUR, 2) + powf(l, 2) - powf(leg->TIBIA, 2)) / (2 * leg->FEMUR * l));

    float phi2 = atan2f(z, h);

    float theta2 = 90  - (phi1 + phi2);

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


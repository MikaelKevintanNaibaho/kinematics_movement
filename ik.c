#include "ik.h"
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include "pwm_servo.h"


// Define a small epsilon value for floating-point comparisons
#define EPSILON 1e-3
#include "ik.h"

const float leg_zero_offset[3] = {0 , 0, 0};


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

void forward_kinematics(SpiderLeg *leg, float sudut[3]) {

    float theta1 = radians(sudut[0]);
    float theta2 = radians(sudut[1]);
    float theta3 = (sudut[2]);

    float phi3 = radians(180 - theta3);

    float H = sqrtf((powf(FEMUR_LENGTH, 2) + powf(TIBIA_LENGTH, 2)) - (2 * FEMUR_LENGTH * TIBIA_LENGTH * cosf(phi3)));

    float phi1 = acosf((powf(FEMUR_LENGTH,2) + powf(H, 2) - powf(TIBIA_LENGTH, 2)) / (2 * FEMUR_LENGTH * H));

    float phi2 = theta2 - phi1;


    float z_coordinate =  H * sinf(phi2);
    float P = (H * cosf(phi2));

    float ya = sinf(theta1) * COXA_LENGTH;
    float xa = cosf(theta1) * COXA_LENGTH;

    float yb = sinf(theta1) * P;
    float xb = cosf(theta1) * P;

    float x_coordinate = xa + xb;
    float y_coordinate = ya + yb;

    leg->joints[3][0] = x_coordinate;
    leg->joints[3][1] = y_coordinate;
    leg->joints[3][2] = z_coordinate;

    printf("forward kinematics: ");
    printf("x = %.2f, y = %.2f, z = %.2f \n", leg->joints[3][0], leg->joints[3][1], leg->joints[3][2]);
    printf("R= %.2f\n phi1= %.2f\n phi2= %.2f\n, P= %.2f\n ", H, phi1, phi3, P);
}


float *get_target(SpiderLeg *leg) {
    return leg->joints[3];
}



void inverse_kinematics(SpiderLeg *leg, float *target) {

    float x = target[0];
    float y = target[1];
    float z = target[2];

    // Calculate theta1 based on the target coordinates
    float theta1 = atan2f(y, x);

    // Print the target coordinates
    printf("koordinat target: \n");
    printf ("x = %.2f\n", x);
    printf ("y = %.2f\n", y);
    printf ("z = %.2f\n", z);

    // Calculate ya and xa based on theta1
    float ya = cosf(theta1) * COXA_LENGTH;
    float xa = sinf(theta1) * COXA_LENGTH;

    printf("xa = %.2f, ya = %.2f\n", xa, ya);

    // Calculate yb and xb based on target coordinates and ya, xa
    float yb  = y - ya;
    float xb  = x - xa;

    printf("xb = %.2f, yb = %.2f\n" , xb, yb);

    // Calculate P
    float P = sqrtf(powf(yb, 2) + powf(xb, 2));
    printf("P = %.2f\n", P);

    float H = sqrtf(powf(z, 2) + powf(P, 2));

    float phi1_cos = (powf(FEMUR_LENGTH, 2) + powf(H, 2) - powf(TIBIA_LENGTH, 2)) / (2 * FEMUR_LENGTH * H);
    float phi1 = acosf(phi1_cos);

    float phi2_cos = (powf(H, 2) + powf(TIBIA_LENGTH, 2) - powf(FEMUR_LENGTH, 2)) / (2 * H * TIBIA_LENGTH);
    float phi2 = acosf(phi2_cos);

    float phi3_cos = (powf(FEMUR_LENGTH, 2) + powf(TIBIA_LENGTH, 2) - powf(H, 2)) / (2 * FEMUR_LENGTH * TIBIA_LENGTH);
    float phi3 = acosf(phi3_cos);
    printf("Phi3 = %.2f\n", degrees(phi3));

    float phi4 = M_PI / 2 - phi2;

    float theta2_relative_to_z = phi1 - phi4;

    float theta2 = phi2 + phi4 + theta2_relative_to_z;

    float total_phi = phi2 + phi4;
    printf("phi2 + phi4 = %.2f + %.2f = %.2f\n", degrees(phi2), degrees(phi4), degrees(total_phi));
    float theta3 = M_PI - (phi3);


    // Convert angles to degrees
    float angles[3] = {degrees(theta1), degrees(theta2), degrees(theta3)};

    // Set the angles for the leg servos
    set_angles(leg, angles);

    // Print the forward kinematics after inverse kinematics
    printf("AFTER INVERSE KINEMATICS\n");
    forward_kinematics(leg, angles);
}








// Function to move a single leg of the hexapod robot forward by distance units
void move_forward(SpiderLeg *leg, float target[3]) {

    // Calculate the target position relative to the leg's current position
    float new_position[3];
    new_position[0] =  target[0] + leg->joints[3][0];  // Update x-coordinate relative to current position
    new_position[1] = target[1] + leg->joints[3][1];   // Update y-coordinate relative to current position
    new_position[2] = target[2] + leg->joints[3][2];   // Update z-coordinate relative to current position
    
    // Update the leg with inverse kinematics to reach the new position
    inverse_kinematics(leg, new_position);
}


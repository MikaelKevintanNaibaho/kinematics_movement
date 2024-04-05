#include "ik.h"
#include <math.h>
#include <stdbool.h>
#include <stdio.h>



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

    printf("Theta1: %.4f degrees\n", leg->theta1);
    printf("Theta2: %.4f degrees\n", leg->theta2);
    printf("Theta3: %.4f degrees\n", leg->theta3);
}

void forward_kinematics(SpiderLeg *leg) {
    double theta1 = radians(leg->theta1);
    double theta2 = radians(leg->theta2);
    double theta3 = radians(leg->theta3);

    double Xa = leg->COXA * cos(theta1);
    double Ya = leg->COXA * sin(theta1);
    double G2 = sin(theta2) * leg->FEMUR;
    double P1 = cos(theta2) * leg->FEMUR;
    double Xc = cos(theta1) * P1;
    double Yc = sin(theta1) * P1;

    double H = sqrt(leg->TIBIA * leg->TIBIA + leg->FEMUR * leg->FEMUR - 2 * leg->TIBIA * leg->FEMUR * cos(M_PI - theta3));
    double phi1 = acos((leg->FEMUR * leg->FEMUR + H * H - leg->TIBIA * leg->TIBIA) / (2 * leg->FEMUR * H));
    double phi2 = M_PI - (M_PI - theta3) - phi1;
    double phi3 = (phi1 - theta2);
    double Pp = cos(phi3) * H;
    double P2 = Pp - P1;
    double Yb = sin(theta1) * Pp;
    double Xb = cos(theta1) * Pp;
    double G1 = -sin(phi3) * H;

    leg->joints[0][0] = 0;
    leg->joints[0][1] = 0;
    leg->joints[0][2] = 0;

    leg->joints[1][0] = Xa;
    leg->joints[1][1] = Ya;
    leg->joints[1][2] = 0;

    leg->joints[2][0] = Xa + Xc;
    leg->joints[2][1] = Ya + Yc;
    leg->joints[2][2] = G2;

    leg->joints[3][0] = Xa + Xb;
    leg->joints[3][1] = Ya + Yb;
    leg->joints[3][2] = G1;
}
float *get_target(SpiderLeg *leg) {
    return leg->joints[3];
}

void inverse_kinematics(SpiderLeg *leg, float *target) {
    float x = target[0];
    float y = target[1];
    float z = target[2];

    printf ("x = %.2f\n", x);
    printf ("y = %.2f\n", y);
    printf ("z = %.2f\n", z);

    float theta1 = atan2f(y, x);

    float ya = leg->COXA * sinf(theta1);

    float xa = leg->COXA * cosf(theta1);

    float yb = x - ya;
    float xb = x - xa;

    float r1 = sqrtf((xb * xb) + (yb * yb));

    float h = sqrtf((r1 * r1) + (z * z));

    // Check if h is zero to avoid division by zero
    if (h == 0) {
        printf("Error: Target position is at the origin.\n");
        return;
    }

    float phi3_cos = (powf(leg->TIBIA, 2) + powf(h, 2) - powf(leg->FEMUR, 2)) / (2 * leg->TIBIA * h);
    // Ensure that phi3_cos is within the valid range [-1, 1]
    if (phi3_cos < -1 || phi3_cos > 1) {
        printf("Error: Invalid cosine value for phi3 calculation.\n");
        return;
    }
    float phi3 = acosf(phi3_cos);

    float phi1_cos = (powf(leg->FEMUR, 2) + powf(h, 2) - powf(leg->TIBIA, 2)) / (2 * leg->FEMUR * h);
    // Ensure that phi1_cos is within the valid range [-1, 1]
    if (phi1_cos < -1 || phi1_cos > 1) {
        printf("Error: Invalid cosine value for phi1 calculation.\n");
        return;
    }
    float phi1 = acosf(phi1_cos);

    float theta2 = phi1 - atan2f(z, r1);

    float theta3 = 180 - (phi1 + phi3);

    // Update the joint angles
    float angles[3] = {degrees(theta1), degrees(theta2), degrees(theta3)};
    set_angles(leg, angles);

        // Update the joint positions directly based on inverse kinematics solution
    leg->joints[3][0] = x + leg->joints[3][0];
    leg->joints[3][1] = y + leg->joints[3][1];
    leg->joints[3][2] = z + leg->joints[3][2];

    // Print the updated joint positions
    printf("Joint positions after inverse kinematics:\n");
    for (int i = 0; i < 4; ++i) {
        printf("Joint %d: (%.2f, %.2f, %.2f)\n", i + 1, leg->joints[i][0], leg->joints[i][1], leg->joints[i][2]);
    }


}



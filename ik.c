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
}

void forward_kinematics(SpiderLeg *leg) {
    float theta1 = radians(leg->theta1);
    float theta2 = radians(leg->theta2);
    float theta3 = radians(leg->theta3);
    

    float Xa = leg->COXA * cos(theta1);
    float Ya = leg->COXA * sin(theta1);
    float G2 = sin(theta2) * leg->FEMUR;
    float P1 = cos(theta2) * leg->FEMUR;
    float Xc = cos(theta1) * P1;
    float Yc = sin(theta1) * P1;

    float H = sqrt(pow(leg->TIBIA, 2) + pow(leg->FEMUR, 2) - (2 * leg->TIBIA * leg->FEMUR * cos(M_PI - theta3)));
    float phi1 = acos((pow(leg->FEMUR, 2) + pow(H, 2) - pow(leg->TIBIA, 2)) / (2 * leg->FEMUR * H));
    float phi2 = M_PI - (M_PI - theta3) - phi1;
    float phi3 = (phi1 - theta2);
    float Pp = cos(phi3) * H;
    float P2 = Pp - P1;
    float Yb = sin(theta1) * Pp;
    float Xb = cos(theta1) * Pp;
    float G1 = -sin(phi3) * H;

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


void inverse_kinematics(SpiderLeg *leg, float *target)
{
    float x = target[0];
    float y = target[1];
    float z = target[2];

    z = fmaxf(z, 0.01);

    float theta1 = atan2f(y, x);

    float xa = leg->COXA * cosf(theta1);
    float ya = leg->COXA * sinf(theta1);

    float xb = x - xa;
    float yb = y - ya;

    float r1 = sqrtf((powf(xb, 2) + powf(yb, 2)));

    float H = sqrtf((powf(r1, 2) + powf(z, 2)));

    float phi2 = atan2f(z, r1);

    float phi3 = 90 -phi2;

    float phi1 = acosf((powf(leg->FEMUR, 2) + powf(H, 2) - powf(leg->TIBIA, 2)) / (2 * leg->TIBIA * H));

    float theta2  = (phi1 - phi2);


    float theta3 = 90 - (phi1 + phi3);

    printf("H: %.2f, phi1: %.2f, phi2: %.2f, phi3: %.2f, theta1: %.2f, theta2: %.2f, theta3: %.2f\n", H, degrees(phi1), degrees(phi2), degrees(phi3), degrees(theta1), degrees(theta2), degrees(theta3));

    float angles[3] = {degrees(theta1), degrees(theta2), degrees(theta3)};
    set_angles(leg, angles);

    forward_kinematics(leg);    
}
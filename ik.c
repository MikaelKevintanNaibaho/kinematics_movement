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
    if (angle > 180.0)
        return angle - 360.0;
    else if (angle < -180.0)
        return angle + 360.0;
    else
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


void inverse_kinematics(SpiderLeg *leg, float *target) {
    float x = target[0];
    float y = target[1];
    float z = target[2];

    float maxX = leg->COXA + leg->FEMUR + leg->TIBIA;
    float minX = -maxX;
    float maxY = leg->FEMUR + leg->TIBIA;
    float minY = 0.0;
    float maxZ = leg->FEMUR + leg->TIBIA;
    float minZ = -maxZ;

    if (x < minX || x > maxX || y < minY || y > maxY || z < minZ || z > maxZ) {
        printf("Error: Target position is out of reach for the leg.\n");
        return;
    }

    float theta1;
    if (fabs(x) < EPSILON) {
        theta1 = (y > 0) ? M_PI / 2.0 : -M_PI / 2.0;
    } else {
        theta1 = atan2(y, x);
    }

    float Xa = leg->COXA * cos(theta1);
    float Ya = leg->COXA * sin(theta1);

    float Xb = x - Xa;
    float Yb = y - Ya;

    float P = sqrt(pow(Xb, 2) + pow(Yb, 2));

    float G = fabs(z);

    float H = sqrt(pow(P, 2) + pow(G, 2));

    if (fabs(H) < EPSILON) {
        printf("Error: Invalid target position for inverse kinematics.\n");
        return;
    }

    float phi3 = asin(G / H);

    float phi2Acos = ((pow(leg->TIBIA, 2)) + (pow(H, 2)) - (pow(leg->FEMUR, 2))) / (2 * leg->TIBIA * H);
    phi2Acos = fminf(fmaxf(phi2Acos, -1.0), 1.0);  // Clamp to valid range for acos
    float phi2 = acos(phi2Acos);

    float phi1 = acos((pow(leg->FEMUR, 2) + pow(H, 2) - pow(leg->TIBIA, 2)) / (2 * leg->FEMUR * H));

    float theta2 = phi1 + phi3;
    float theta3 = phi1 + phi2;

    // Check for invalid angles near singularities
    if (isnan(theta2) || isnan(theta3)) {
        printf("Error: Unable to calculate valid joint angles for the target position.\n");
        return;
    }

    float angles[3] = {degrees(theta1), degrees(theta2), degrees(theta3)};
    set_angles(leg, angles);
    forward_kinematics(leg);
}

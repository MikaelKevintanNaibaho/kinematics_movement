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

    // Calculate theta1
    float theta1 = atan2f(y, x);

    // Calculate the projected distance on the x-y plane
    float projected_distance = sqrtf(x * x + y * y);

    // If the target only specifies x and y (z = 0), use the current z-coordinate
    if (fabs(z) < EPSILON) {
        // If z-coordinate is close to zero, use the current leg position for z
        z = leg->joints[3][2];
    }

    // Calculate the remaining distances
    float remaining_distance = z - leg->joints[3][2];

    // Calculate the total distance to the target
    float total_distance = sqrtf(projected_distance * projected_distance + remaining_distance * remaining_distance);

    // Calculate theta2
    float cos_theta2 = (powf(total_distance, 2) - powf(leg->FEMUR, 2) - powf(leg->TIBIA, 2)) / (2 * leg->FEMUR * leg->TIBIA);
    float theta2 = acosf(cos_theta2);

    // Calculate theta3
    float cos_theta3 = (powf(leg->FEMUR, 2) + powf(leg->TIBIA, 2) - powf(total_distance, 2)) / (2 * leg->FEMUR * leg->TIBIA);
    float theta3 = acosf(cos_theta3);

    // Set the calculated angles
    float angles[3] = {degrees(theta1), degrees(theta2), degrees(theta3)};
    set_angles(leg, angles);

    // Print the results or use them as needed
    printf("Theta1: %.4f degrees\n", leg->theta1);
    printf("Theta2: %.4f degrees\n", leg->theta2);
    printf("Theta3: %.4f degrees\n", leg->theta3);

    // Perform forward kinematics to update joint positions
    forward_kinematics(leg);

    // Debug print statements
    printf("x: %.4f\n", x);
    printf("y: %.4f\n", y);
    printf("z: %.4f\n", z);
    printf("Theta1: %.4f degrees\n", degrees(theta1));
    printf("Theta2: %.4f degrees\n", degrees(theta2));
    printf("Theta3: %.4f degrees\n", degrees(theta3));
}

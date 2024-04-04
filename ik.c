#include <stdio.h>
#include "ik.h"

double to_degrees(double radians)
{
    return radians * (180.0 / PI);
}
double to_radians(double degrees)
{
    return degrees * (PI / 180.0);
}

void set_angles(SpiderLeg *leg, double angles[3])
{
    // Normalize angles to be in range [0, 180] degrees
    for (int i = 0; i < 3; i++) {
        double angle = angles[i];

        // Normalize angle to be within [0, 180] degrees
        while (angle > 180.0) {
            angle -= 180.0;
        }
        while (angle < 0.0) {
            angle += 180.0;
        }

        angles[i] = angle;
    }

    // Assign angles to the leg
    leg->theta1 = angles[0];
    leg->theta2 = angles[1];
    leg->theta3 = angles[2];
}

void forward_kinematics(SpiderLeg *leg)
{
    double theta1 = to_radians(leg->theta1);
    double theta2 = to_radians(leg->theta2);
    double theta3 = to_radians(leg->theta3);

    // Forward kinematics calculation
    double Xa = 0.0;  // X-coordinate of joint 1
    double Ya = 0.0;  // Y-coordinate of joint 1
    double G1 = 0.0;  // Z-coordinate of joint 1

    double Xb = leg->COXA * cos(theta1);  // X-coordinate of joint 2
    double Yb = leg->COXA * sin(theta1);  // Y-coordinate of joint 2
    double G2 = 0.0;  // Z-coordinate of joint 2

    double Xc = Xb + leg->FEMUR * cos(theta1 + theta2);  // X-coordinate of joint 3
    double Yc = Yb + leg->FEMUR * sin(theta1 + theta2);  // Y-coordinate of joint 3
    double G3 = -leg->FEMUR * sin(theta2);  // Z-coordinate of joint 3

    double Xd = Xc + leg->TIBIA * cos(theta1 + theta2 + theta3);  // X-coordinate of joint 4
    double Yd = Yc + leg->TIBIA * sin(theta1 + theta2 + theta3);  // Y-coordinate of joint 4
    double G4 = G3;  // Z-coordinate of joint 4

    // Store joint positions in leg's joints array
    leg->joints[0][0] = Xa;
    leg->joints[0][1] = Ya;
    leg->joints[0][2] = G1;

    leg->joints[1][0] = Xb;
    leg->joints[1][1] = Yb;
    leg->joints[1][2] = G2;

    leg->joints[2][0] = Xc;
    leg->joints[2][1] = Yc;
    leg->joints[2][2] = G3;

    leg->joints[3][0] = Xd;
    leg->joints[3][1] = Yd;
    leg->joints[3][2] = G4;
}


void inverse_kinematics(SpiderLeg *leg, double target[3])
{
    double x = target[0];
    double y = target[1];
    double z = target[2];

    // Calculate theta1 using arctangent
    double theta1 = atan2(y, x);

    // Intermediate values
    double Xa = leg->COXA * cos(theta1);
    double Ya = leg->COXA * sin(theta1);

    double Xb = x - Xa;
    double Yb = y - Ya;

    double P = Xb / cos(theta1);
    double G = fabs(z);
    double H = sqrt(pow(P, 2) + pow(G, 2));

    double phi3 = asin(G / H);

    double phi2Acos = (pow(leg->TIBIA, 2) + pow(H, 2) - pow(leg->FEMUR, 2)) / (2 * leg->TIBIA * H);
    double phi2 = acos(phi2Acos);

    double phi1 = acos((pow(leg->FEMUR, 2) + pow(H, 2) - pow(leg->TIBIA, 2)) / (2 * leg->FEMUR * H));

    double theta2, theta3;
    if (z > 0) {
        theta2 = phi1 + phi3;
    } else {
        theta2 = phi1 - phi3;
    }
    theta3 = phi1 + phi2;

    double angles[3] = {to_degrees(theta1), to_degrees(theta2), to_degrees(theta3)};
    set_angles(leg, angles);
    forward_kinematics(leg);
}


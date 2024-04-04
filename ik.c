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

void set_angles(SpiderLeg *leg, double angles[3]) {
    // Normalize angles to be in the range [-180, 180] degrees
    for (int i = 0; i < 3; i++) {
        double ang = angles[i];
        int sign = 1;
        if (ang < 0) {
            sign = -1;
        }
        angles[i] = sign * (fabs(ang) - (360.0 * floor(fabs(ang) / 360.0)));
        if (fabs(ang) > 180.0) {
            angles[i] = angles[i] - (360.0 * sign);
        }
    }
    leg->theta1 = angles[0];
    leg->theta2 = angles[1];
    leg->theta3 = angles[2];
}


void forward_kinematics(SpiderLeg *leg) {
    double theta1 = to_radians(leg->theta1) - 90;
    double theta2 = to_radians(leg->theta2) - 90;
    double theta3 = to_radians(leg->theta3) -90;

    // Forward kinematics calculations
    double Xa = leg->COXA * cos(theta1);
    double Ya = leg->COXA * sin(theta1);
    double G2 = sin(theta2) * leg->FEMUR;
    double P1 = cos(theta2) * leg->FEMUR;
    double Xc = cos(theta1) * P1;
    double Yc = sin(theta1) * P1;

    // Position of the tip of the leg (end effector)
    double H = sqrt(pow(leg->TIBIA, 2) + pow(leg->FEMUR, 2) - (2 * leg->TIBIA * leg->FEMUR * cos(PI - theta3)));
    double phi1 = acos((pow(leg->FEMUR, 2) + pow(H, 2) - pow(leg->TIBIA, 2)) / (2 * leg->FEMUR * H));
    double phi2 = PI - (PI - theta3) - phi1;
    double phi3 = (phi1 - theta2);
    double Pp = cos(phi3) * H;
    double P2 = Pp - P1;
    double Yb = sin(theta1) * Pp;
    double Xb = cos(theta1) * Pp;
    double G1 = -sin(phi3) * H;

    // Store joint positions in leg's joints array
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


void inverse_kinematics(SpiderLeg *leg, double target[3]) {
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

  // Check for reachability using leg limits
  double max_reach23 = leg->FEMUR + leg->TIBIA; // Combined maximum reach of joints 2 and 3
  if (H > max_reach23) {
    printf("target unreachabel\n");
    // Target unreachable, handle the case (e.g., return error code)
    return; // Replace with your desired unreachable target handling
  }

  double phi3 = asin(G / H);

  // Limit calculation of phi2 based on reachability
  double phi2Acos = (pow(leg->TIBIA, 2) + pow(H, 2) - pow(leg->FEMUR, 2)) / (2 * leg->TIBIA * H);
  double phi2;
  if (phi2Acos <= 1 && phi2Acos >= 0) {
    phi2 = acos(phi2Acos);
  } else {
    // Target partially reachable, adjust phi2 to reach limit
    phi2 = max_reach23 - phi3; // Adjust based on your desired behavior (e.g., prioritize reaching z)
  }

  double phi1 = acos((pow(leg->FEMUR, 2) + pow(H, 2) - pow(leg->TIBIA, 2)) / (2 * leg->FEMUR * H));

  double theta2, theta3;
  if (z > 0) {
    theta2 = to_degrees(phi1 + phi3 );
  } else {
    theta2 = to_degrees(phi1 - phi3 );
  }
  theta3 = to_degrees(phi1 + phi2 );

  theta1 = to_degrees(theta1) ;

  double angles[3] = {theta1, theta2, theta3};
  set_angles(leg, angles);
  forward_kinematics(leg);
}


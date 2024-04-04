#ifndef IK_H
#define IK_H

#include <math.h>
// #include "pwm_servo.h"


#define PI 3.141559265359

typedef struct {
    char name[20];
    double COXA;
    double FEMUR;
    double TIBIA;
    double theta1;
    double theta2;
    double theta3;
    double joints[4][3]; // [x, y, z] coordinates of each joint
} SpiderLeg;

double to_degrees(double radians);
double to_radians(double degrees);

void set_angles(SpiderLeg *leg, double angles[3]);
void forward_kinematics(SpiderLeg *leg);
void inverse_kinematics(SpiderLeg *leg, double target[3]);


#endif /*IK_H*/
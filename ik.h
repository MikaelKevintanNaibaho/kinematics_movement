#ifndef IK_H
#define IK_H

#include <math.h>
// #include "pwm_servo.h"


#define M_PI 3.141559265359

typedef struct {
    char name[20];
    float COXA;
    float FEMUR;
    float TIBIA;
    float theta1;
    float theta2;
    float theta3;
    float joints[4][3]; // Joint positions: [0] - start joint, [1] - coxa-femur joint, [2] - femur-tibia joint, [3] - tip of the leg
} SpiderLeg;

float to_degrees(float radians);
float to_radians(float degrees);
float normalize_angle(float angle);
float *get_target(SpiderLeg *leg);

void set_angles(SpiderLeg *leg, float angles[3]);
void forward_kinematics(SpiderLeg *leg);
void inverse_kinematics(SpiderLeg *leg, float *target);


#endif /*IK_H*/
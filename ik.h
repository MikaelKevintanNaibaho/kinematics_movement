#ifndef IK_H
#define IK_H

#include <math.h>
#include "pwm_servo.h"

#define COXA_LENGTH 75.0
#define FEMUR_LENGTH 80.0
#define TIBIA_LENGTH 170.0
#define INITIAL_ANGLE2 90
#define INITIAL_ANGLE3 90
#define PI 3.141559265359

void calculate_ik(float x, float y, float z, float* angles);
void move_leg(float startx, float starty, float startz, float endx, float endy, float endz, int steps, int freq);

#endif /*IK_H*/
#ifndef IK_H
#define IK_H

#include <math.h>
#include "pwm_servo.h"

#define COXA_LENGTH 75.0
#define FEMUR_LEGTH 80.0
#define TIBIA_LEGTH 170.0
#define PI 3.141559265359

void calculate_ik(float x, float y, float z, float* angles);
void move_leg(float startx, float starty, float startz, float endx, float endy, float endz, int steps, int freq);

#endif /*IK_H*/
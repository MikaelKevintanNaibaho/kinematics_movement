#ifndef IK_H
#define IK_H

#include <math.h>
#include <stdio.h>
#include <string.h>
#include "pca9685.h"
#include "dh.h"
#include "leg.h"

#define DELTA_THETA_MAX 1
#define DELAY_US 1000
#define PWM_FREQ 50

float degrees(float rad);
float radians(float deg);
float normalize_angle(float angle);
float *get_target(SpiderLeg *leg);
float calculate_delta_theta(int speed);

void set_angles(SpiderLeg *leg, const float angles[3]);
void calculate_delta_direction(const float target_angles[3], const float current_angles[3],
                               float delta_directions[3]);
void compute_dh_params(DHParameters params_array[NUM_LINKS], float theta1, float theta2,
                       float theta3);
void forward_kinematics(SpiderLeg *leg, const float angles[3], LegPosition position_leg);
void inverse_kinematics(SpiderLeg *leg, const float target_positions[3], LegPosition position_leg);

void move_to_angle(SpiderLeg *leg, float target_angles[3], int speed);
int angles_equal(const float angles1[3], const float angles2[3]);

// coordinates
void adjust_coordinate(float x, float y, float z, LegPosition position, float *adj_x, float *adj_y,
                       float *adj_z);
void adjust_angle(float theta1, float theta2, float theta3, LegPosition position, float *adj_theta1,
                  float *adj_theta2, float *adj_theta3);

#endif /*IK_H*/

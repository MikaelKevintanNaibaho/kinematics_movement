#ifndef IK_H
#define IK_H

#include <math.h>
#include <stdio.h>
#include "pwm_servo.h"
#include "matrix.h"
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_vector.h>

#define NUM_LINKS 4

#define SERVO_CHANNEL_1 1
#define SERVO_CHANNEL_2 2
#define SERVO_CHANNEL_3 3

#define COXA_LENGTH  60.4
#define FEMUR_LENGTH 78.0
#define TIBIA_LENGTH 167.23 


#define PWM_FREQ 50
typedef struct {
    char name[20];
    float COXA;
    float FEMUR;
    float TIBIA;
    float theta1;
    float theta2;
    float theta3;
    float mounted_angle;
    float joints[4][3]; // Joint positions: [0] - start joint, [1] - coxa-femur joint, [2] - femur-tibia joint, [3] - tip of the leg
} SpiderLeg;

typedef struct {
    float alpha;
    float a;
    float d;
    float theta;
}DHParameters;

typedef struct {
    float matrix[4][4];
}DHMatrix;

// Define error codes
typedef enum {
    IK_SUCCESS = 0,
    IK_ERROR_LIMIT_REACHED,
    IK_ERROR_INVALID_INPUT,
    // Add more error codes as needed
} IK_ErrorCode;



extern const float leg_zero_offset[3];


float to_degrees(float rad);
float to_radians(float deg);
float normalize_angle(float angle);
float *get_target(SpiderLeg *leg);

void set_angles(SpiderLeg *leg, float angles[3]);
void forward_kinematics(SpiderLeg *leg, float angles[3], gsl_matrix *intermediate_matrices[]);
void inverse_kinematics(SpiderLeg *leg, float target_position[3], gsl_matrix *intermediate_metrices[]);
void move_forward(SpiderLeg *leg, float target[3]);
void handle_error(IK_ErrorCode error_code) ;

//DH
void init_DH_params(DHParameters *params, float alpha, float a, float d, float theta);
void create_DH_matrix(const DHParameters *params, gsl_matrix *matrix);
void print_DH_matrix(const DHMatrix *matrix);
void multiply_DH_matrices(const DHMatrix *matrix1, const DHMatrix *matrix2, DHMatrix *result);
void calculate_DH_transformation(const DHParameters *params_array, int num_links, gsl_matrix *result, gsl_matrix *intermediate_matrices[]);
void move_to_angle(SpiderLeg *leg, float target_angles[3], float velocity);

#endif /*IK_H*/ 
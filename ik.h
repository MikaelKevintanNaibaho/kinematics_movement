#ifndef IK_H
#define IK_H

#include <math.h>
#include <stdio.h>
#include "pwm_servo.h"

#define M_PI 3.141559265359
#define NUM_LINKS 4
// #define DEBUG_MATRIX

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

typedef struct {
    float matrix[3][3];
}JacobMatrix;


// Define error codes
typedef enum {
    IK_SUCCESS = 0,
    IK_ERROR_LIMIT_REACHED,
    IK_ERROR_INVALID_INPUT,
    // Add more error codes as needed
} IK_ErrorCode;

typedef struct {
    DHMatrix matrix;
    int link_number;
} LinkTransformation;



extern const float leg_zero_offset[3];


float degrees(float rad);
float radians(float deg);
float normalize_angle(float angle);
float *get_target(SpiderLeg *leg);

void set_angles(SpiderLeg *leg, float angles[3]);
void forward_kinematics(SpiderLeg *leg, float angles[3], LinkTransformation *link_transformations);
void inverse_kinematics(SpiderLeg *leg, float target_position[3],LinkTransformation *link_transformations);
void move_forward(SpiderLeg *leg, float target[3]);
void handle_error(IK_ErrorCode error_code) ;

//DH
void init_DH_params(DHParameters *params, float alpha, float a, float d, float theta);
void create_DH_matrix(const DHParameters *params, DHMatrix *matrix);
void print_DH_matrix(const DHMatrix *matrix);
void multiply_DH_matrices(const DHMatrix *matrix1, const DHMatrix *matrix2, DHMatrix *result);
void calculate_DH_transformation(const DHParameters *params_array, int num_links, DHMatrix *result, LinkTransformation *link_transformations);
void move_to_angle(SpiderLeg *leg, float target_angles[3], float velocity);


#endif /*IK_H*/
#include "ik.h"
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include "pwm_servo.h"


// Define a small epsilon value for floating-point comparisons
#define EPSILON 1e-3
#include "ik.h"

const float leg_zero_offset[3] = {0 , 0, 0};


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

    set_pwm_angle(SERVO_CHANNEL_1, (int)leg->theta1, PWM_FREQ);
    set_pwm_angle(SERVO_CHANNEL_2, (int)leg->theta2,PWM_FREQ );
    set_pwm_angle(SERVO_CHANNEL_3, (int)leg->theta3, PWM_FREQ);

    printf("Theta1: %.4f degrees\n", leg->theta1);
    printf("Theta2: %.4f degrees\n", leg->theta2);
    printf("Theta3: %.4f degrees\n", leg->theta3);
}

void forward_kinematics(SpiderLeg *leg, float sudut[3]) {

    float theta1 = radians(sudut[0]);
    float theta2 = radians(sudut[1]);
    float theta3 = radians(sudut[2]);

    float phi4 = 180 - theta3;

    float R = sqrtf((powf(FEMUR_LENGTH, 2) + powf(TIBIA_LENGTH, 2)) - (2 * FEMUR_LENGTH * TIBIA_LENGTH * cosf(theta3)));

    float phi1 = acosf((powf(FEMUR_LENGTH,2) + powf(R, 2) - powf(TIBIA_LENGTH, 2)) / (2 * FEMUR_LENGTH * R));

    float phi3 =  phi1 - theta2;

    float z_coordinate = R * sinf(phi3);
    float H = R * cosf(phi3);

    float G = COXA_LENGTH + H;

    float x_coordinate = G * sinf(theta1);
    float y_coordinate = G * cosf(theta1);

    leg->joints[3][0] = x_coordinate;
    leg->joints[3][1] = y_coordinate;
    leg->joints[3][2] = z_coordinate;
    printf("forward kinematics: ");
    printf("x = %.2f, y = %.2f, z = %.2f \n", leg->joints[3][0], leg->joints[3][1], leg->joints[3][2]);
}

float *get_target(SpiderLeg *leg) {
    return leg->joints[3];
}

void inverse_kinematics(SpiderLeg *leg, float *target) {

    float x = target[0];
    float y = target[1];
    float z = target[2];

    float theta1;
    
    if (x != 0 ){
        theta1 = atan2f(x, y);

    } else{
        theta1 = radians(leg->theta1);
    }

    if (z == 0) {
        z = leg->joints[3][2];
    }

    printf("koordinat target: \n");
    printf ("x = %.2f\n", x);
    printf ("y = %.2f\n", y);
    printf ("z = %.2f\n", z);

 



    float ya = cosf(theta1) * COXA_LENGTH;
    float xa = sinf(theta1) * COXA_LENGTH;

    printf("xa = %.2f, ya = %2.f\n", xa, ya);

    float yb  = y - ya;
    float xb  = x - xa;


    printf("xb = %.2f, yb = %.2f\n" , xb, yb);
    

    float H = sqrtf(powf(yb, 2) + powf( xb, 2));

    float R = sqrtf(powf(H, 2) + powf(z, 2));

    float phi1 = acosf((powf(FEMUR_LENGTH,2) + powf(R, 2) - powf(TIBIA_LENGTH, 2)) / (2 * FEMUR_LENGTH * R));

    float phi3 = atan2f(z, H);

    float theta2 = 90 + (phi1 - phi3);

    float phi4 = acosf((powf(FEMUR_LENGTH, 2) + powf(TIBIA_LENGTH, 2) - powf(R, 2)) / (2 * FEMUR_LENGTH * TIBIA_LENGTH));

    float theta3 = 180 - phi4;


    float angles[3] = {degrees(theta1), degrees(theta2), degrees(theta3)};

    set_angles(leg, angles);
    forward_kinematics(leg, angles);

}







// Function to move a single leg of the hexapod robot forward by distance units
void move_forward(SpiderLeg *leg, float target[3]) {

    // Calculate the new target position after moving forward
    float new_position[3];
    new_position[0] =  target[0];  // Update x-coordinate
    new_position[1] = target[1];
    new_position[2] = target[2];
    
    // Update the leg with inverse kinematics to reach the new position
    inverse_kinematics(leg, new_position);
}


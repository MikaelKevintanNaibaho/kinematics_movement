#include <stdio.h>
#include <unistd.h>
#include "pwm_servo.h"
#include "ik.h"

#define COXA_LENGTH 75.0
#define FEMUR_LENGTH 80.0
#define TIBIA_LENGTH 170.0
#define PWM_FREQ 50

#define SERVO_CHANNEL_1 1
#define SERVO_CHANNEL_2 2
#define SERVO_CHANNEL_3 3

int main(void)
{
    // Initialize SpiderLeg with initial joint angles
    SpiderLeg leg1 = {"leg1", COXA_LENGTH, FEMUR_LENGTH, TIBIA_LENGTH, 90.0, 30.0, 0.0, {{0}}};

    // Perform forward kinematics to compute initial joint positions
    forward_kinematics(&leg1);

    printf("Joint positions after forward kinematics (default position):\n");
    for (int i = 0; i < 4; i++)
    {
        printf("Joint %d: (%.2f, %.2f, %.2f)\n", i + 1, leg1.joints[i][0], leg1.joints[i][1], leg1.joints[i][2]);
    }

    // Define target position for inverse kinematics
    double target[3] = {leg1.joints[3][0] + 30, leg1.joints[3][1], leg1.joints[3][2]};

    // Perform inverse kinematics to move to the target position
    inverse_kinematics(&leg1, target);

    printf("\nJoint positions after inverse kinematics (target position):\n");
    for (int i = 0; i < 4; i++)
    {
        printf("Joint %d: (%.2f, %.2f, %.2f)\n", i + 1, leg1.joints[i][0], leg1.joints[i][1], leg1.joints[i][2]);
    }

    set_pwm_angle(SERVO_CHANNEL_1, leg1.theta1, PWM_FREQ);
    set_pwm_angle(SERVO_CHANNEL_2, leg1.theta2, PWM_FREQ);
    set_pwm_angle(SERVO_CHANNEL_3, leg1.theta3, PWM_FREQ);

    return 0;
}

#include <stdio.h>
#include <unistd.h>
#include "pwm_servo.h"
#include "ik.h"

#define COXA_LENGTH 65.0 
#define FEMUR_LENGTH 75.0 
#define TIBIA_LENGTH 165.0 
#define PWM_FREQ 50

#define SERVO_CHANNEL_1 1
#define SERVO_CHANNEL_2 2
#define SERVO_CHANNEL_3 3

#define NUM_JOINTS 4

#include <stdio.h>
#include "ik.h"

int main() {

    PCA9685_init();
    // Create a SpiderLeg object
    SpiderLeg leg;

    // Set leg dimensions
    leg.COXA = COXA_LENGTH;  // Example value, replace with your actual values
    leg.FEMUR = FEMUR_LENGTH;
    leg.TIBIA = TIBIA_LENGTH;

    // Set initial joint angles (example values)
    leg.theta1 = 90.0;
    leg.theta2 = 30.0;
    leg.theta3 = 30.0;

    set_pwm_angle(SERVO_CHANNEL_1, leg.theta1, PWM_FREQ);
    set_pwm_angle(SERVO_CHANNEL_2, leg.theta2, PWM_FREQ);
    set_pwm_angle(SERVO_CHANNEL_3, leg.theta3, PWM_FREQ);

    // Perform forward kinematics to calculate joint positions
    forward_kinematics(&leg);

    // Display joint positions after forward kinematics
    printf("Joint positions after forward kinematics:\n");
    for (int i = 0; i < NUM_JOINTS; ++i) {
        printf("Joint %d: (%.2f, %.2f, %.2f)\n", i + 1, leg.joints[i][0], leg.joints[i][1], leg.joints[i][2]);
    }

    while (1)
    {
        float target[3];

        target[0] += 100.0;

        inverse_kinematics(&leg, target);

         // Display joint angles after inverse kinematics
        printf("\nAngles after inverse kinematics:\n");
        printf("Theta 1: %.2f degrees\n", leg.theta1);
        printf("Theta 2: %.2f degrees\n", leg.theta2);
        printf("Theta 3: %.2f degrees\n", leg.theta3);

        // Perform forward kinematics again to verify the calculated joint angles
        forward_kinematics(&leg);

        // Display joint positions after applying inverse kinematics
        printf("\nJoint positions after inverse kinematics:\n");
        for (int i = 0; i < NUM_JOINTS; ++i) {
            printf("Joint %d: (%.2f, %.2f, %.2f)\n", i + 1, leg.joints[i][0], leg.joints[i][1], leg.joints[i][2]);
        }

        set_pwm_angle(SERVO_CHANNEL_1, leg.theta1, PWM_FREQ);
        set_pwm_angle(SERVO_CHANNEL_2, leg.theta2, PWM_FREQ);
        set_pwm_angle(SERVO_CHANNEL_3, leg.theta3, PWM_FREQ);

        usleep(100000);
    }
    

    

    return 0;
}


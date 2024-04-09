#include <stdio.h>
#include <unistd.h>
#include "pwm_servo.h"
#include "ik.h"





int main() {
    // Initialize the PCA9685 PWM controller
    PCA9685_init();

    // Define leg parameters
    SpiderLeg leg;
    leg.COXA = COXA_LENGTH;   // Example dimensions, replace with actual values
    leg.FEMUR = FEMUR_LENGTH;  // Example dimensions, replace with actual values
    leg.TIBIA = TIBIA_LENGTH;  // Example dimensions, replace with actual values // Example PWM channel, replace with actual channel number

    // Set initial joint angles for the leg
    float initial_angles[3] = {45, 150.0, 130.0};  // Example angles, replace with desired initial angles


    set_angles(&leg, initial_angles);
    forward_kinematics(&leg, initial_angles);
    sleep(2);
    // Move the leg forward by a specified distance
    float target[3] = {-50.0, -50.0, 0.0}; // Example distance to move forward
    move_forward(&leg, target);

    return 0;


}


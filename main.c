#include "ik.h"
#include "pwm_servo.h"

int main(void){
    PCA9685_init();

    SpiderLeg leg;
    float target[3] = {100.0, 100.0, 0.0};

    inverse_kinematics(&leg, target);

    sleep(2);
    float lift_height = -20.0;
    float step_lenght = 50;
    float num_step = 3;

    for (int i = 0; i < num_step; i++){
        //lift the leg
        float target_lift[3] = {0.0, 0.0, lift_height};
        inverse_kinematics(&leg, target_lift);
        sleep(1);
        //move forward
        float target_forward[3] = {step_lenght, 0.0, lift_height};
        inverse_kinematics(&leg, target_forward);
        sleep(1);
        //palce down
        float target_down[3] = {step_lenght, 0.0, 0.0};
        inverse_kinematics(&leg, target_down);
        sleep(1);
        //move backward
        float target_backward[3] = {0.0, 0.0, 0.0};
        inverse_kinematics(&leg, target_backward);
        sleep(1);
    }

    return 0;
}
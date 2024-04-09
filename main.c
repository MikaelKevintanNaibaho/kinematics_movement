#include "ik.h"
#include "pwm_servo.h"

int main(void){
    PCA9685_init();

    SpiderLeg leg;
    float target[3] = {0.0, 0.0, 0.0};

    inverse_kinematics(&leg, target);

    sleep(2);
    float lift_height = 20.0;
    float step_lenght = 50;
    float num_step = 3;

    for (int i = 0; i < num_step; i++){
        //lift the leg
        float target_lift[3] = {0.0, 0.0, lift_height};
        inverse_kinematics(&leg, target_lift);
        sleep(0.5);
        //move forward
        float target_forward[3] = {step_lenght, 0.0, lift_height};
        inverse_kinematics(&leg, target_forward);
        sleep(0.5);
        //palce down
        float target_down[3] = {step_lenght, 0.0, 0.0};
        inverse_kinematics(&leg, target_down);
        sleep(0.5);
        //move backward
        float target_backward[3] = {-step_lenght, 0.0, 0.0};
        inverse_kinematics(&leg, target_backward);
        sleep(0.5);
    }

    return 0;
}
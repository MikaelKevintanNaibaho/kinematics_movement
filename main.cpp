#include "hexapod.h"

void set_angle_servo_pwm (hexapod &robot){
    for(int servo = 0; servo < 18; servo++){
        float angle = robot.servoangle[servo] * RADTODEG;
        set_pwm_angle(servo, angle, SERVO_FREQ);
        printf("servo %d = %.2f\n", servo, robot.servoangle[servo]);
    }
}

int main(void) {
    PCA9685_init();

    hexapod walker(true);

    walker.angle[0] = 45 * DEGTORAD;
    walker.angle[1] = 150 * DEGTORAD;
    walker.angle[2] = 130 * DEGTORAD;



    walker.stand();
    set_angle_servo_pwm(walker);


    return 0;
}
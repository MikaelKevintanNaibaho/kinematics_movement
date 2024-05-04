#include "ik.h"
#include "move.h"
#include "pwm_servo.h"

int main(void)
{
    // Initialize PCA9685 if necessary
    PCA9685_init();

    initialize_all_legs();

    // Set initial angles using forward kinematics
    stand_position();
    sleep(2);

    move_forward();

    return 0;
}

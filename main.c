#include "ik.h"
#include "move.h"
#include "pwm_servo.h"
#include "interrupt.h"

int main(void)
{
    // Initialize PCA9685 if necessary
    PCA9685_init();

    initialize_all_legs();

    // Set initial angles using forward kinematics
    stand_position();

    init_interrupt();

    while (1)
    {
        if(is_program_running) {
            move_forward();
        } else{
            return 0;
        }
    }
    

    return 0;
}

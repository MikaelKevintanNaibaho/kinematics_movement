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

    while (!is_program_running)
    {
        // You can add a delay here to avoid high CPU usage
        delay(100);
    }

    while (is_program_running)
    {
        move_forward();
        switch_interrupt();
    }
    

    return 0;
}

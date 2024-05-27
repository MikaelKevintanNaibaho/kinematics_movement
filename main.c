#include "ik.h"
#include "move.h"
#include "pwm_servo.h"
#include "interrupt.h"
#include "capit.h"



// int main(void)
// {
//     // Initialize PCA9685 if necessary
//     PCA9685_init();

//     initialize_all_legs();

//     // Set initial angles using forward kinematics
//     stand_position();

//     init_interrupt();

//     while (1) {
//         // Check if the switch is turned on
//         if (is_program_running) {
//             // If the switch is on, move forward
//             move_forward();     
//             // move_left_turn();
//             // move_left_turn();
//         } else {
//             stand_position();
//             // If the switch is off, pause the program
//             // You can add additional functionality here if needed
//             // For example, you can keep the robot in its current position
//             // Or you can add logic to respond to other inputs or events
//         }

//         // Add a small delay to avoid high CPU usage
//         delay(100);
//     }

//     /*testing turn left relative functions*/
//     // move_left_turn();

//     return 0;
// }

int main(void)
{
    PCA9685_init();

    set_angle_sg90(30);
    sleep(2);
    set_angle_sg90(50);
    sleep(2);
    set_angle_sg90(70);

    return 0;
}
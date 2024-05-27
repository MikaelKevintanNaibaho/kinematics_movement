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
int main() {
    // Initialize the PCA9685 module
    PCA9685_init();

    // Set the PWM frequency to 50Hz for servos
    set_pwm_freq(50);

    // Test SG90 servo on channel 0
    int angle;

    for (angle = 0; angle <= 90; angle += 30) {
        set_angle_mg(CAPIT_BASE, angle);
        printf("Set angle to %d degrees\n", angle);
        sleep(1); // Wait for 1 second
    }

    // for (angle = 90; angle >= 0; angle -= 30) {
    //     set_sg90_angle(CAPIT_BASE, angle);
    //     printf("Set angle to %d degrees\n", angle);
    //     sleep(1); // Wait for 1 second
    // }

    return 0;
}
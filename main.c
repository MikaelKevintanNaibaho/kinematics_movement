#include "ik.h"
#include "move.h"
#include "pwm_servo.h"
#include "interrupt.h"

void test_inverse_kinematics_for_all_legs(float target_position[3])
{
    // Initialize all legs
    initialize_all_legs();

    // Define leg positions
    LegPosition leg_positions[NUM_LEGS] = { KANAN_DEPAN, KIRI_DEPAN, KIRI_BELAKANG, KANAN_BELAKANG };

    // Perform inverse kinematics for each leg
    for (int i = 0; i < NUM_LEGS; i++) {
        printf("Testing Inverse Kinematics for Leg %s:\n", legs[i]->name);
        inverse_kinematics(legs[i], target_position, leg_positions[i]);
        printf("---------------------------------------\n");
    }
}

int main(void)
{
    // Initialize PCA9685 if necessary
    PCA9685_init();

    initialize_all_legs();

    // Set initial angles using forward kinematics
    stand_position();

    init_interrupt();

    while (1) {
        // Check if the switch is turned on
        if (is_program_running) {
            // If the switch is on, move forward
            move_forward();     
            // move_left_turn();
            // move_left_turn();
        } else {
            stand_position();
            // If the switch is off, pause the program
            // You can add additional functionality here if needed
            // For example, you can keep the robot in its current position
            // Or you can add logic to respond to other inputs or events
        }

        // Add a small delay to avoid high CPU usage
        delay(100);
    }

    /*testing turn left relative functions*/
    // move_left_turn();

    return 0;
}

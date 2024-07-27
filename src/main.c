#include "ik.h"
#include "move.h"
#include "i2c_utils.h"
#include "pca9685.h"
#include "interrupt.h"

int main(void)
{
    I2CInterface *i2c_iface = get_i2c_interface();
    if (i2c_iface == NULL) {
        fprintf(stderr, "I2C interface not set.\n");
        return -1;
    }

    // Initialize PCA9685 if necessary
    pca9685_init(i2c_iface);
    initialize_all_legs();

    // Set initial angles using forward kinematics
    stand_position();

    // init_interrupt();

    while (1) {
        // Check if the switch is turned on
        if (is_program_running) {
            // If the switch is on, move forward
            move_forward();
        } else {
            stand_position();
            // If the switch is off, pause the program
            // You can add additional functionality here if needed
            // For example, you can keep the robot in its current position
            // Or you can add logic to respond to other inputs or events
        }
    }

    return 0;
}

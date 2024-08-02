#include "move.h"
#include "pca9685.h"
#include "interrupt.h"
#include <unistd.h>
#include <wiringPi.h>

/* define MOCK untuk compile dan run tanpa harware untuk bypass i2c dan wiringPi
 * define REAL untuk compile dan run dengan hardware
 * */

#define MOCK
// #define REAL

#ifdef REAL
#include "i2c_utils.h"
#include "gpio_utils.h"
#endif

#ifdef MOCK
#include "mock_i2c.h"
#include "mock_gpio.h"
#endif

int main(void)
{
#ifdef MOCK
    set_mock_i2c();
    set_mock_gpio_interface();
#endif
#ifdef REAL
    set_real_i2c();
    set_real_gpio();
#endif
    pca9685_init();
    initialize_all_legs();

    // Set initial angles using forward kinematics
    stand_position();

    init_interrupt();
#ifdef MOCK
    set_mock_pin_state(LOW);
#endif
    while (1) {
        if (is_program_running) {
            move_forward();
        } else {
            stand_position();
        }
    }
    return 0;
}

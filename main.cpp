#include "hexapod.h"

int main() {

    PCA9685_init();
     // Create a hexapod object with debug mode enabled for informative messages
    set_pwm_angle(1, 90, 50);
    sleep(1);
    set_pwm_angle(1, 0, 50);

  return 0;
}

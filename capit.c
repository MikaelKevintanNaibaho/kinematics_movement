#include "capit.h"

void set_sg90_angle(uint8_t channel, int angle)
{
    if (angle < 0) {
        angle = 0;
    } else if (angle > 180) {
        angle = 180;
    }

    int pulse_width = 500 + ((2500 - 500) * angle / 180);
    set_pwm_duty(channel, pulse_width);
}

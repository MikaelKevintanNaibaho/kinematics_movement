#include "capit.h"

void set_pwm_angle_mg(uint8_t channel, int angle, int freq) {
    if (angle < 0) {
        angle = 0;
    } else if (angle > 180) {
        angle = 180;
    }

    int pulse_width = MIN_PULSE_WIDTH_MG960 + ((MAX_PULSE_WIDTH_MG960 - MIN_PULSE_WIDTH_MG960) * angle / 180);

    set_pwm_duty(channel, pulse_width);

    set_pwm_freq(freq);
}


void set_angle_mg(uint8_t channel, int angle) {
    if (angle < 0) {
        angle = 0;
    } else if (angle > 180) {
        angle = 180;
    }

    int pulse_width = 1000 + ((2000 - 1000) * angle / 180);
    set_pwm_duty(channel, pulse_width);
}
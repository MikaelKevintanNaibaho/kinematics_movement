#include "capit.h"

int map_angle_to_duty_sg90(int angle) {
    int duty_cycle = 0;
    if (angle < 0) {
        angle = - angle;
        duty_cycle = CENTER_PULSE_WIDTH - (angle * NEGATIVE_PULSE_WIDTH_DIFF) / 90;
    } else {
        duty_cycle = CENTER_PULSE_WIDTH + (angle * POSITIVE_PULSE_WIDTH_DIFF) / 90;
    }

    return duty_cycle;
}

void set_pwm_angle_sg90(uint8_t channel, int angle, int freq)
{
    int duty_cyle = map_angle_to_duty_sg90(angle);

    set_pwm_duty(channel, duty_cyle);

    set_pwm_freq(freq);
}

void set_angle_sg90(int angle)
{
   set_pwm_angle_sg90(CAPIT_UJUNG, angle, FREQ);
}



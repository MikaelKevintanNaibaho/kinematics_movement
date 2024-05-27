#ifndef CAPIT_H
#define CAPIT_H

#include "pwm_servo.h"

//channel
#define CAPIT_BASE 13
#define CAPIT_UJUNG 14

#define FREQ 50
//pulse
#define CENTER_PULSE_WIDTH 1500
#define POSITIVE_PULSE_WIDTH_DIFF 200
#define NEGATIVE_PULSE_WIDTH_DIFF 200


int map_angle_to_duty_sg90(int angle);
void set_pwm_angle_sg90(uint8_t channel, int angle, int freq);
void set_angle_sg90(int angle);



#endif //CAPIT_H
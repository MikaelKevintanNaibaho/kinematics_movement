#ifndef CAPIT_H
#define CAPIT_H

#include "pwm_servo.h"

//channel
#define CAPIT_BASE 13
#define CAPIT_UJUNG 14
#define FREQ 50

#define MIN_PULSE_WIDTH_MG960 1000  // 1 ms
#define MAX_PULSE_WIDTH_MG960 2000 // 2 ms

void set_pwm_angle_mg(uint8_t channel, int angle, int freq) ;
void set_angle_mg(uint8_t channel, int angle);

#endif //CAPIT_H
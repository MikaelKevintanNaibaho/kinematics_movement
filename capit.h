#ifndef CAPIT_H
#define CAPIT_H

#include "pwm_servo.h"

//channel
#define CAPIT_BASE 13
#define CAPIT_UJUNG 14
#define FREQ 50

void set_sg90_angle(uint8_t channel, int angle);



#endif //CAPIT_H
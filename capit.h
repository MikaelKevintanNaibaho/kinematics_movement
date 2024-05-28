#ifndef CAPIT_H
#define CAPIT_H

#include "pwm_servo.h"

//channel
#define CAPIT_BASE 13
#define CAPIT_UJUNG 14
#define FREQ 50

void set_angle_mg(int angle);
void set_angle_sg(int angle);

void buka_capit(void);
void tutup_capit(void);
void turun_capit(void);
void naik_capit(void);

void capit(void);
void letak(void);

#endif //CAPIT_H
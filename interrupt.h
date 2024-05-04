#ifndef INTERRUPT_H
#define INTERRUPT_H

#include <stdio.h>
#include <wiringPi.h>

#define SWITCH_PIN 17

extern int is_program_running;

void init_interrupt(void);
void start_program(void);
void stop_program(void);

#endif /* INTERRUPT_H */

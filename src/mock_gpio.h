#ifndef MOCK_GPIO_H
#define MOCK_GPIO_H

#include "gpio_interface.h"
#include <pthread.h>

static pthread_mutex_t state_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t state_cond = PTHREAD_COND_INITIALIZER;

static void mock_wiringPiSetupGpio();
static void mock_pinMode(int pin, int mode);
static void mock_pullUpDnControl(int pin, int pud);
static int mock_digitalRead(int pin);
void set_mock_gpio_interface(void);
void set_mock_pin_state(int state);

#endif // MOCK_GPIO_H

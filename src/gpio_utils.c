#include "gpio_interface.h"
#include <wiringPi.h>
#include "gpio_utils.h"

static void gpio_wiringPiSetupGpio()
{
    wiringPiSetupGpio();
}

static void gpio_pinMode(int pin, int mode)
{
    pinMode(pin, mode);
}

static void gpio_pullUpDnControl(int pin, int pud)
{
    pullUpDnControl(pin, pud);
}

static int gpio_digitalRead(int pin)
{
    return digitalRead(pin);
}

static int gpio_wiringPiISR(int pin, int mode, void (*f)(void))
{
    return wiringPiISR(pin, mode, f);
}

static GPIOInterface real_gpio_interface = { .wiringPiSetupGpio = gpio_wiringPiSetupGpio,
                                             .pinMode = gpio_pinMode,
                                             .pullUpDnControl = gpio_pullUpDnControl,
                                             .digitalRead = gpio_digitalRead,
                                             .wiringPiISR = gpio_wiringPiISR };

void set_real_gpio(void)
{
    set_gpio_interface(&real_gpio_interface);
}

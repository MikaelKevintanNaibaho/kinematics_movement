#include "mock_gpio.h"
#include "gpio_interface.h"
#include <stdio.h>


int mock_digital_read_state = 0;
static void (*mock_interrupt_handler)(void) = NULL;

static void mock_wiringPiSetupGpio() { }
static void mock_pinMode(int pin, int mode) { }
static void mock_pullUpDnControl(int pin, int pud) { }
static int mock_digitalRead(int pin)
{
        return mock_digital_read_state;
}


// Mock GPIO ISR
static int mock_wiringPiISR(int pin, int mode, void (*f)(void))
{
    mock_interrupt_handler = f;
    return 0;
}

static GPIOInterface mock_gpio_interface = { .wiringPiSetupGpio = mock_wiringPiSetupGpio,
                                             .pinMode = mock_pinMode,
                                             .pullUpDnControl = mock_pullUpDnControl,
                                             .digitalRead = mock_digitalRead,
                                             .wiringPiISR = mock_wiringPiISR };
// Function to get the mock GPIO interface
GPIOInterface *get_mock_gpio_interface(void)
{
    return &mock_gpio_interface;
}

void set_mock_gpio_interface()
{
    set_gpio_interface(&mock_gpio_interface);
}

void set_mock_pin_state(int state)
{
    mock_digital_read_state = state;
    if (mock_interrupt_handler) {
        mock_interrupt_handler();
    }
}



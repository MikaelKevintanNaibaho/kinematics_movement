#include <wiringPi.h>
#include "interrupt.h"
#include "gpio_interface.h"
#include "mock_gpio.h"
#include <stdio.h>

int is_program_running = 0;

void switch_interrupt()
{
    if (get_gpio_interface() == NULL) {
        fprintf(stderr, "Error: gpio_iface is NULL in switch_interrupt\n");
        return;
    }
    int pin_state = get_gpio_interface()->digitalRead(SWITCH_PIN);
    printf("Interrupt: pin state = %d\n", pin_state); // Debug print
    if (pin_state == LOW) {
        if (!is_program_running) {
            start_program();
        }
    } else {
        if (is_program_running) {
            stop_program();
        }
    }
}

void init_interrupt(void)
{
    GPIOInterface *gpio_iface = get_gpio_interface();
    gpio_iface->wiringPiSetupGpio();
    gpio_iface->pinMode(SWITCH_PIN, INPUT);
    gpio_iface->pullUpDnControl(SWITCH_PIN, PUD_UP);
    gpio_iface->wiringPiISR(SWITCH_PIN, INT_EDGE_FALLING, &switch_interrupt);
}

void start_program(void)
{
    printf("Starting program ...\n");
    is_program_running = 1;
    pthread_cond_signal(&state_cond); // Signal main thread to start
}

void stop_program(void)
{
    printf("Stopping program ...\n");
    is_program_running = 0;
}

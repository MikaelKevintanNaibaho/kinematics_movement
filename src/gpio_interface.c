#include <stddef.h>
#include "gpio_interface.h"

GPIOInterface *gpio_iface = NULL;

// function to set the gpio interface
void set_gpio_interface(GPIOInterface *iface)
{
    gpio_iface = iface;
}

GPIOInterface *get_gpio_interface(void)
{
    return gpio_iface;
}

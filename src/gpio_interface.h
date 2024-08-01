#ifndef GPIO_INTERFACE_H
#define GPIO_INTERFACE_H

#include <stdint.h>

#ifdef __cplusplus
exrtern "C"
{
#endif

    typedef struct
    {
        void (*wiringPiSetupGpio)();
        void (*pinMode)(int pin, int mode);
        void (*pullUpDnControl)(int pin, int pud);
        int (*digitalRead)(int pin);
        int (*wiringPiISR)(int pin, int mode, void (*f)(void));
    } GPIOInterface;

    extern GPIOInterface *gpio_interface;
    void set_gpio_interface(GPIOInterface * iface);
    GPIOInterface *get_gpio_interface(void);

#ifdef __cplusplus
}
#endif

#endif // GPIO_INTERFACE_H

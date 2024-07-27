#include "i2c_interface.h"
#include <stddef.h>
// Define the global variable for the I2C interface
I2CInterface *i2c_iface = NULL;

// Function to set the I2C interface
void set_i2c_interface(I2CInterface *iface)
{
    i2c_iface = iface;
}

I2CInterface *get_i2c_interface(void)
{
    return i2c_iface;
}

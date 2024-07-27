#ifndef I2C_INTERFACE_H
#define I2C_INTERFACE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    int (*open)(const char *device);
    int (*set_slave_address)(uint8_t addr);
    void (*close)();
    int (*write_byte)(uint8_t reg, uint8_t val);
    uint8_t (*read_byte)(uint8_t reg);
} I2CInterface;

extern I2CInterface *i2c_iface;
void set_i2c_interface(I2CInterface *iface);
I2CInterface *get_i2c_interface(void);
#ifdef __cplusplus
}
#endif

#endif // I2C_INTERFACE_H

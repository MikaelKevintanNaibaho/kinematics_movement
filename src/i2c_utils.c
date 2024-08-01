// i2c_utils.c
#include "i2c_utils.h"
#include "i2c_interface.h"
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <unistd.h>

static int i2c_fd = -1;

static int i2c_open(const char *device)
{
    i2c_fd = open(device, O_RDWR);
    if (i2c_fd < 0) {
        perror("Failed to open i2c device");
        return -1;
    }
    return 0;
}

static void i2c_close()
{
    if (i2c_fd >= 0) {
        close(i2c_fd);
        i2c_fd = -1;
    }
}

static int i2c_set_slave_address(uint8_t addr)
{
    if (ioctl(i2c_fd, I2C_SLAVE, addr) < 0) {
        perror("Failed to set i2c slave address");
        return -1;
    }
    return 0;
}

static int i2c_write_byte(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    if (write(i2c_fd, buf, 2) != 2) {
        perror("Error writing byte");
        return -1;
    }
    return 0;
}

static uint8_t i2c_read_byte(uint8_t reg)
{
    uint8_t buf[1];
    if (write(i2c_fd, &reg, 1) != 1) {
        perror("Error writing register address");
        return 0;
    }

    if (read(i2c_fd, buf, 1) != 1) {
        perror("Error reading byte");
        return 0;
    }
    return buf[0];
}

// Define the default I2C interface
static I2CInterface real_i2c_interface = { .open = i2c_open,
                                           .set_slave_address = i2c_set_slave_address,
                                           .close = i2c_close,
                                           .write_byte = i2c_write_byte,
                                           .read_byte = i2c_read_byte };

void set_real_i2c(void)
{
    set_i2c_interface(&real_i2c_interface);
}

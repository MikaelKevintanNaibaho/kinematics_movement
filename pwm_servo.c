#include "pwm_servo.h"

int open_i2_device()
{
    i2c_fd = open(I2C_DEVICE, O_RDWR);
    if (i2c_fd < 0) {
        perror("Error opening i2c device");
        return -1;
    }

    return 0;
}
int init_pca9685()
{
    uint8_t buf[2];
    //set pca9685 to sleep mode
    buf[0] = MODE1_REG;
    buf[1] = 0x10; //sleep
    if (write(i2c_fd, buf, 2) != 2) {
        perror("Error writing to pca9685");
        return -1;
    }

    buf[0] = PRE_SCALE;
    buf[1] = 0x79 ;//50hz
    if (write(i2c_fd, buf, 2) != 2) {
        perror("Error writing to pac9685");
        return -1;
    }

    //wake pca9685 from sleep mode
    buf[0] = MODE1_REG;
    buf[0] = 0x00;
       if (write(i2c_fd, buf, 2) != 2) {
        perror("Error writing to pac9685");
        return -1;
    }

    return 0;
}
void set_servo_position(int channel, int position)
{
    uint8_t buf[5];
    uint16_t on_time = (uint16_t)(position * 4);

    buf[0] = LED0_ON_L + 4 * channel;
    buf[1] = 0x00; //on time low byte
    buf[2] = 0x00; //on time high byte
    buf[3] = on_time & 0xFF; //off time low byte
    buf[4] = (on_time) >> 8;

    write(i2c_fd, buf, 5);

}

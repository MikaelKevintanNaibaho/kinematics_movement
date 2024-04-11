#include "pwm_servo.h"

int i2c_fd;

void PCA9685_init()
{
    i2c_fd = open(I2C_DEVICE, O_RDWR);
    if (i2c_fd < 0) {
        perror("Failed to open the i2c device");
        return;
    }

    if (ioctl(i2c_fd, I2C_SLAVE, PCA9685_SLAVE_ADDR) < 0) {
        perror("Error to set i2c address");
        close(i2c_fd);
        return;
    }

    //init
    write_byte(MODE1, 0x00);
    write_byte(MODE2, 0x04);
}

void write_byte(uint8_t reg, uint8_t val)
{
    uint8_t buf[2];
    buf[0] = reg;
    buf[1] = val;
    if(write(i2c_fd, buf, 2) != 2) {
        perror("Error writing byte");
        return;
    }
}

uint8_t read_byte(uint8_t reg) {
    uint8_t buf[1];
    buf[0] = reg;
    if (write(i2c_fd, buf, 1) != 1) {
        perror("Write failed");
        return 0;
    }
    if (read(i2c_fd, buf, 1) != 1) {
        perror("Read failed");
        return 0;
    }
    return buf[0];
}

void set_pwm_freq(int freq)
{
    uint8_t prescale_val = (uint8_t)((CLOCK_FREQ / 4096 * freq) - 1);
    write_byte(MODE1, 0x10); //sleep
    write_byte(PRE_SCALE, prescale_val);
    write_byte(MODE1, 0x80); //restart
    write_byte(MODE2, 0x04); //totem pole (default)
}
void set_pwm_duty(uint8_t led, int pulse_width)
{
    set_pwm(led, 0, pulse_width);
}


void set_pwm(uint8_t led, int on_value, int off_value)
{
    write_byte(LED0_ON_L + LED_MULTIPLIER * (led - 1), on_value & 0xFF);
    write_byte(LED0_ON_L + LED_MULTIPLIER * (led - 1) + 1, on_value >> 8);
    write_byte(LED0_OFF_L + LED_MULTIPLIER * (led - 1), off_value & 0xFF);
    write_byte(LED0_OFF_L + LED_MULTIPLIER * (led - 1) + 1, off_value >> 8);
}


int get_pwm(uint8_t led)
{
    int led_value = 0;
    led_value = read_byte(LED0_OFF_L + LED_MULTIPLIER * (led - 1));
    led_value |= (read_byte(LED0_OFF_L + LED_MULTIPLIER * (led - 1) + 1) << 8);

    return led_value;
}

void set_pwm_angle(uint8_t channel, int angle, int freq, float velocity)
{   
    if (angle < 0) {
        angle = 0;
    } else if (angle > 180) {
        angle = 180;
    }

    int current_angle = read_current_angle(channel);
    int increment = velocity * 100.0;
    while(current_angle != angle){
        if (current_angle < angle) current_angle += increment;
        else if (current_angle > angle) current_angle -= increment;
        else current_angle = angle;

        set_pwm_duty(channel, theta(current_angle));
        set_pwm_freq(freq);
        usleep(10000);
    }
}

int read_current_angle(uint8_t channel){
    int pulse_width = get_pwm(channel);
    int current_angle = inverse_theta(pulse_width);
    return current_angle;
}

int inverse_theta(int pulse_width) {
    int compere;
    int min_comp = MIN_PULSE_WIDTH;
    int max_comp = MAX_PULSE_WIDTH;
    int min_angle = 0;
    int max_angle = 180;
    
    // Invert the calculation of theta function to get the angle
    compere = (pulse_width - min_comp) / ((max_comp - min_comp) / (max_angle - min_angle)) + min_angle;
    return compere;
}


int theta(int angle){
    int compere;
    int min_comp = MIN_PULSE_WIDTH;
    int max_comp = MAX_PULSE_WIDTH;
    int min_angle = 0; 
    int max_angle = 180;
    compere = (max_comp-min_angle) / (max_angle- min_angle) * (angle - min_angle) + min_comp;
    return compere;
}
 
#include "calibrate_servo.h"

void set_zero(void) {
    set_pwm_angle(SERVO_CHANNEL_1, 0);
    set_pwm_angle(SERVO_CHANNEL_2, 0);
    set_pwm_angle(SERVO_CHANNEL_3, 0);
    set_pwm_angle(SERVO_CHANNEL_4, 0);
    set_pwm_angle(SERVO_CHANNEL_5, 0);
    set_pwm_angle(SERVO_CHANNEL_6, 0);
    set_pwm_angle(SERVO_CHANNEL_7, 0);
    set_pwm_angle(SERVO_CHANNEL_8, 0);
    set_pwm_angle(SERVO_CHANNEL_9, 0);
    set_pwm_angle(SERVO_CHANNEL_10, 0);
    set_pwm_angle(SERVO_CHANNEL_11, 0);
    set_pwm_angle(SERVO_CHANNEL_12, 0);
}

void set_pwm_angle_manual(uint8_t channel, int pulse_width)
{
    set_pwm_duty(channel, pulse_width);
}

void calibrate_servo(uint8_t channel)
{
    int min_pulse_width, max_pulse_width;
    char done;
    do {
        // Set servo to minimum position
        printf("Set servo to minimum position and enter pulse width: ");
        min_pulse_width = read_int_from_terminal();
        set_pwm_angle_manual(channel, min_pulse_width);

        // Set servo to maximum position
        printf("Set servo to maximum position and enter pulse width: ");
        max_pulse_width = read_int_from_terminal();
        set_pwm_angle_manual(channel, max_pulse_width);

        // Print the calibration values
        printf("Channel %d - Min Pulse Width: %d, Max Pulse Width: %d\n", channel, min_pulse_width, max_pulse_width);
        printf("Are you satisfied with these values? (y/n): ");
        done = read_char_from_terminal();
    } while (done != 'y' && done != 'Y');
}

void adjust_servo(uint8_t channel, int angle)
{
    if (angle < 0) {
        angle = 0;
    } else if (angle > 180) {
        angle = 180;
    }

    int min_pulse_width = calibration_data[channel - 1].min_pulse_width;
    int max_pulse_width = calibration_data[channel - 1].max_pulse_width;

    int pulse_width = min_pulse_width + ((max_pulse_width - min_pulse_width) * angle / 180);
    set_pwm_duty(channel, pulse_width);
}

int read_int_from_terminal()
{
    int value;
    scanf("%d", &value);
    return value;
}

char read_char_from_terminal()
{
    char value;
    scanf(" %c", &value);
    return value;
}

int main(void) {
    PCA9685_init();

    set_zero();

    for (int i = 1; i <= 12; i++) {
        calibrate_servo(i);
    }

    return 0;
}

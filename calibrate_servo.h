#ifndef CALIBRATE_SERVO_H
#define CALIBRATE_SERVO_H

#include "pwm_servo.h"

#define SERVO_CHANNEL_1 1
#define SERVO_CHANNEL_2 2
#define SERVO_CHANNEL_3 3
#define SERVO_CHANNEL_4 4
#define SERVO_CHANNEL_5 5
#define SERVO_CHANNEL_6 6
#define SERVO_CHANNEL_7 7
#define SERVO_CHANNEL_8 8
#define SERVO_CHANNEL_9 9
#define SERVO_CHANNEL_10 10
#define SERVO_CHANNEL_11 11
#define SERVO_CHANNEL_12 12

struct CalibrationData
{
    int min_pulse_width;
    int max_pulse_width;
}calibration_data[12];

void set_zero(void);
void set_pwm_angle_manual(uint8_t channel, int pulse_width);
void calibrate_servo(uint8_t channel);
int read_int_from_terminal();
char read_char_from_terminal();

#endif //CALIBRATE_SERVO_H
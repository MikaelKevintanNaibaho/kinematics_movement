#ifndef TEST_PCA9685_H
#define TEST_PCA9685_H

#include <unity/unity.h>

void test_pca9685_init(void);
void test_set_pwm_freq(void);
void test_set_pwm_duty(void);
void test_set_pwm(void);
void test_get_pwm(void);
void test_set_pwm_angle(void);

#endif // TEST_PCA9685_H

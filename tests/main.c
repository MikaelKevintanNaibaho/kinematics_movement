// main.c
#include <unity/unity.h>
#include <unity/unity_internals.h>
#include "test_bezier2d.h"
#include "test_bezier3d.h"
#include "test_i2c.h"
#include "test_pca9685.h"
#include "test_ik.h"
#include "test_dh.h"

int main(void)
{
    UNITY_BEGIN();

    // Run all the Bezier2D tests
    RUN_TEST(test_bezier2d_init);
    RUN_TEST(test_bezier2d_addPoint);
    RUN_TEST(test_bezier2d_getPos);
    RUN_TEST(test_bezier2d_generate_curve);

    // Run all the Bezier3D tests
    RUN_TEST(test_bezier3d_init);
    RUN_TEST(test_bezier3d_addpoint);
    RUN_TEST(test_bezier3d_getpos);
    RUN_TEST(test_bezier3d_generate_curve);

    // Run all the I2C tests
    RUN_TEST(test_i2c_open);
    RUN_TEST(test_i2c_set_slave_address);
    RUN_TEST(test_i2c_write_byte);
    RUN_TEST(test_i2c_read_byte);

    // Runn all pca9685 test
    RUN_TEST(test_pca9685_init);
    RUN_TEST(test_set_pwm_freq);
    RUN_TEST(test_set_pwm);
    RUN_TEST(test_set_pwm_duty);
    RUN_TEST(test_get_pwm);
    RUN_TEST(test_set_pwm_angle);

    // Run all denavit-hattenberg DH test
    RUN_TEST(test_init_DH_params);
    RUN_TEST(test_create_DH_matrix);
    RUN_TEST(test_calculate_DH_transformation);

    // Run all inverse kinematics test
    RUN_TEST(test_degress);
    RUN_TEST(test_radians);
    RUN_TEST(test_normalize_angle);
    RUN_TEST(test_get_target);
    RUN_TEST(test_set_angles);
    RUN_TEST(test_angles_equal);
    RUN_TEST(test_calculate_delta_theta);
    RUN_TEST(test_calculate_delta_direction);
    RUN_TEST(test_move_to_angle);
    RUN_TEST(test_compute_dh_params);
    RUN_TEST(test_forward_kinematics);
    RUN_TEST(test_inverse_kinematics);
    return UNITY_END();
}

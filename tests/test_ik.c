#include "test_ik.h"
#include <stdio.h>
#include <unity/unity.h>
#include "ik.h"
#include "leg.h"

#define PI 3.14159265358979323846

#define TEST_ASSERT_FLOAT_WITHIN_TOLERANCE(expected, actual, tolerance, msg)                       \
    do {                                                                                           \
        if (fabs((expected) - (actual)) > (tolerance)) {                                           \
            printf("%s: Expected %f but got %f\n", (msg), (expected), (actual));                   \
            TEST_FAIL();                                                                           \
        }                                                                                          \
    } while (0)

void test_degress(void)
{
    float tolerance = 0.0001;
    TEST_ASSERT_FLOAT_WITHIN_TOLERANCE(0.0, degrees(0.0), tolerance, "Conversion of 0 rad failed");
    TEST_ASSERT_FLOAT_WITHIN_TOLERANCE(45.0, degrees(PI / 4), tolerance,
                                       "Conversion of PI / 4  rad failed");
    TEST_ASSERT_FLOAT_WITHIN_TOLERANCE(90.0, degrees(PI / 2), tolerance,
                                       "Conversion of PI / 2 rad failed");
    TEST_ASSERT_FLOAT_WITHIN_TOLERANCE(180.0, degrees(PI), tolerance,
                                       "Conversion of PI rad failed");
}

void test_radians(void)
{
    float tolerance = 0.0001;
    TEST_ASSERT_FLOAT_WITHIN_TOLERANCE(0.0, radians(0.0), tolerance,
                                       "Conversion of 0 degrees failed");
    TEST_ASSERT_FLOAT_WITHIN_TOLERANCE(PI / 4, radians(45.0), tolerance,
                                       "Conversion of 45 degrees failed");
    TEST_ASSERT_FLOAT_WITHIN_TOLERANCE(PI / 2, radians(90.0), tolerance,
                                       "Conversion of 90 degrees failed");
    TEST_ASSERT_FLOAT_WITHIN_TOLERANCE(PI, radians(180.0), tolerance,
                                       "Conversion of 180 degrees failed");
}

void test_normalize_angle(void)
{
    float tolerance = 0.0001;

    TEST_ASSERT_FLOAT_WITHIN_TOLERANCE(0.0, normalize_angle(0.0), tolerance,
                                       "Normalization of 0 degrees failed");
    TEST_ASSERT_FLOAT_WITHIN_TOLERANCE(90.0, normalize_angle(90.0), tolerance,
                                       "Normalization of 90 degrees failed");
    TEST_ASSERT_FLOAT_WITHIN_TOLERANCE(180.0, normalize_angle(180.0), tolerance,
                                       "Normalization of 180 degrees failed");
    TEST_ASSERT_FLOAT_WITHIN_TOLERANCE(90.0, normalize_angle(270.0), tolerance,
                                       "Normalization of 270 degrees failed");
    TEST_ASSERT_FLOAT_WITHIN_TOLERANCE(0.0, normalize_angle(360.0), tolerance,
                                       "Normalization of 360 degrees failed");

    TEST_ASSERT_FLOAT_WITHIN_TOLERANCE(90.0, normalize_angle(-90.0), tolerance,
                                       "Normalization of -90 degrees failed");
    TEST_ASSERT_FLOAT_WITHIN_TOLERANCE(180.0, normalize_angle(-180.0), tolerance,
                                       "Normalization of -180 degrees failed");
    TEST_ASSERT_FLOAT_WITHIN_TOLERANCE(90.0, normalize_angle(-270.0), tolerance,
                                       "Normalization of -270 degrees failed");
    TEST_ASSERT_FLOAT_WITHIN_TOLERANCE(0.0, normalize_angle(-360.0), tolerance,
                                       "Normalization of -360 degrees failed");

    TEST_ASSERT_FLOAT_WITHIN_TOLERANCE(45.0, normalize_angle(405.0), tolerance,
                                       "Normalization of 405 degrees failed");
    TEST_ASSERT_FLOAT_WITHIN_TOLERANCE(135.0, normalize_angle(495.0), tolerance,
                                       "Normalization of 495 degrees failed");
}

void test_get_target(void)
{
    SpiderLeg leg1;
    float expected_values[3] = { 135.0, 180.0, 45.0 };

    // Initialize the joints array
    leg1.joints[3][0] = expected_values[0];
    leg1.joints[3][1] = expected_values[1];
    leg1.joints[3][2] = expected_values[2];

    // Get the target from the function
    float *leg2 = get_target(&leg1);

    // Check if leg2 points to the correct position
    TEST_ASSERT_EQUAL_PTR(leg1.joints[3], leg2);

    // Verify the values pointed to by leg2
    for (int i = 0; i < 3; ++i) {
        TEST_ASSERT_EQUAL_FLOAT(expected_values[i], leg2[i]);
    }
}

void test_set_angles(void)
{
    SpiderLeg leg;
    float expected_angles[3] = { 45.0, 135.0, 90.0 };
    float angles[3] = { 45.0, 135.0, 90.0 };

    set_angles(&leg, angles);

    TEST_ASSERT_EQUAL_FLOAT(expected_angles[0], leg.theta1);
    TEST_ASSERT_EQUAL_FLOAT(expected_angles[1], leg.theta2);
    TEST_ASSERT_EQUAL_FLOAT(expected_angles[2], leg.theta3);
}

void test_angles_equal(void)
{
    float angles1[3] = { 45.0, 135.0, 90.0 };
    float angles2[3] = { 45.0, 135.0, 90.0 };
    float angles3[3] = { 45.0, 130.0, 90.0 };

    TEST_ASSERT_TRUE(angles_equal(angles1, angles2));
    TEST_ASSERT_FALSE(angles_equal(angles1, angles3));
}

void test_calculate_delta_theta(void)
{

    // Test with different speeds
    int speeds[] = { 0, 1, 10, 20 };
    float expected[] = { 0.0, 1.0, 10.0, 20.0 };

    for (int i = 0; i < 4; i++) {
        TEST_ASSERT_FLOAT_WITHIN(0.0001, expected[i], calculate_delta_theta(speeds[i]));
    }
}

void test_calculate_delta_direction(void)
{
    float target_angles[3] = { 90.0, 180.0, 45.0 };
    float current_angles[3] = { 45.0, 135.0, 90.0 };
    float delta_directions[3];

    calculate_delta_direction(target_angles, current_angles, delta_directions);

    // Check the delta directions for each joint
    TEST_ASSERT_FLOAT_WITHIN(0.0001, 1.0, delta_directions[0]); // 90 > 45
    TEST_ASSERT_FLOAT_WITHIN(0.0001, 1.0, delta_directions[1]); // 180 > 135
    TEST_ASSERT_FLOAT_WITHIN(0.0001, -1.0, delta_directions[2]); // 45 < 90
}

void test_move_to_angle(void)
{
    SpiderLeg leg;
    float initial_angles[3] = { 0.0, 0.0, 0.0 };
    float target_angles[3] = { 30.0, 60.0, 90.0 };
    int speed = 10;

    // Set initial angles
    leg.theta1 = initial_angles[0];
    leg.theta2 = initial_angles[1];
    leg.theta3 = initial_angles[2];

    move_to_angle(&leg, target_angles, speed);

    TEST_ASSERT_FLOAT_WITHIN(0.01, target_angles[0], leg.theta1);
    TEST_ASSERT_FLOAT_WITHIN(0.01, target_angles[1], leg.theta2);
    TEST_ASSERT_FLOAT_WITHIN(0.01, target_angles[2], leg.theta3);
}

void test_compute_dh_params(void)
{
    float tolerance = 0.0001;

    DHParameters params_array[NUM_LINKS];
    float theta1 = radians(45.0); // Example angles in degrees
    float theta2 = radians(90.0);
    float theta3 = radians(90.0);

    // Compute DH parameters
    compute_dh_params(params_array, theta1, theta2, theta3);

    // Check expected values
    TEST_ASSERT_FLOAT_WITHIN(tolerance, radians(90.0), params_array[0].alpha);
    TEST_ASSERT_FLOAT_WITHIN(tolerance, COXA_LENGTH, params_array[0].a);
    TEST_ASSERT_FLOAT_WITHIN(tolerance, 0.0, params_array[0].d);
    TEST_ASSERT_FLOAT_WITHIN(tolerance, theta1 + radians(90.0), params_array[0].theta);

    TEST_ASSERT_FLOAT_WITHIN(tolerance, radians(0.0), params_array[1].alpha);
    TEST_ASSERT_FLOAT_WITHIN(tolerance, FEMUR_LENGTH, params_array[1].a);
    TEST_ASSERT_FLOAT_WITHIN(tolerance, 0.0, params_array[1].d);
    TEST_ASSERT_FLOAT_WITHIN(tolerance, theta2, params_array[1].theta);

    TEST_ASSERT_FLOAT_WITHIN(tolerance, radians(-90.0), params_array[2].alpha);
    TEST_ASSERT_FLOAT_WITHIN(tolerance, TIBIA_LENGTH, params_array[2].a);
    TEST_ASSERT_FLOAT_WITHIN(tolerance, 0.0, params_array[2].d);
    TEST_ASSERT_FLOAT_WITHIN(tolerance, theta3 - radians(90.0), params_array[2].theta);

    TEST_ASSERT_FLOAT_WITHIN(tolerance, radians(90.0), params_array[3].alpha);
    TEST_ASSERT_FLOAT_WITHIN(tolerance, 0.0, params_array[3].a);
    TEST_ASSERT_FLOAT_WITHIN(tolerance, 0.0, params_array[3].d);
    TEST_ASSERT_FLOAT_WITHIN(tolerance, radians(-90.0), params_array[3].theta);
}

void test_forward_kinematics(void)
{
    SpiderLeg leg;
    leg.theta1 = 0.0;
    leg.theta2 = 0.0;
    leg.theta3 = 0.0;
    const float angles[3] = { 45.0, 90.0, 90.0 };
    float expected_coor[3] = { 97.86, 97.86, -167.23 };

    // Assert end-effector positions are not NULL
    const LegPosition positions[4] = { KANAN_DEPAN, KIRI_DEPAN, KIRI_BELAKANG, KANAN_BELAKANG };
    for (int i = 0; i < 4; ++i) {
        forward_kinematics(&leg, angles, positions[i]);

        TEST_ASSERT_NOT_NULL(&leg);
        // Check if end-effector position is not NULL or NaN
        for (int j = 0; j < 3; ++j) {
            TEST_ASSERT_FALSE(isnan(leg.joints[3][j]));
        }

        // Check expected coordinates for each position (set expected values accordingly)
        TEST_ASSERT_FLOAT_WITHIN(0.01, expected_coor[0], leg.joints[3][0]);
        TEST_ASSERT_FLOAT_WITHIN(0.01, expected_coor[1], leg.joints[3][1]);
        TEST_ASSERT_FLOAT_WITHIN(0.01, expected_coor[2], leg.joints[3][2]);
    }
}

void test_inverse_kinematics(void)
{
    SpiderLeg leg;
    leg.theta1 = 0.0;
    leg.theta2 = 0.0;
    leg.theta3 = 0.0;
    const float angles[3] = { 45.0, 90.0, 90.0 };

    // Assert end-effector positions are not NULL
    const LegPosition positions[4] = { KANAN_DEPAN, KIRI_DEPAN, KIRI_BELAKANG, KANAN_BELAKANG };
    for (int i = 0; i < 4; ++i) {
        forward_kinematics(&leg, angles, positions[i]);
    }

    float x = leg.joints[3][0];
    float y = leg.joints[3][1];
    float z = leg.joints[3][2];

    float pindah = 20.0;

    float target_positions[3] = { x + pindah, y + pindah, z + pindah };

    for (int i = 0; i < 4; i++) {
        const float expected_theta[3] = { 45.0, 102.78, 92.37 };
        inverse_kinematics(&leg, target_positions, positions[i]);

        float theta[3] = { leg.theta1, leg.theta2, leg.theta3 };
        // check theta
        for (int j = 0; j < 3; j++) {
            TEST_ASSERT_FLOAT_WITHIN(0.01, expected_theta[j], theta[j]);
        }
    }
}

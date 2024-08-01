#include "test_trajectory.h"
#include <stdbool.h>
#include <unity/unity.h>
#include "bezier.h"
#include "common.h"
#include "trajectory.h"

void test_calculate_swing_stance_phase_front(void)
{
    float startx = 97.86;
    float startz = 97.86;
    float stride_length = 50.0;
    float swing_height = 20.0;
    float endx, endz, controlx, controlz;
    bool is_swing_phase = true;
    struct bezier2d *curve = get_curve2d();

    calculate_swing_stance_phase(startx, startz, stride_length, swing_height, FRONT, &controlx,
                                 &controlz, &endx, &endz, is_swing_phase);
    bezier2d_generate_curve(curve, startx, startz, controlx, controlz, endx, endz);

    float expected_controlx = startx + stride_length / 2;
    float expected_controlz = startz + 2 * swing_height;
    float expected_endx = startx + stride_length;
    float expected_endz = startz;

    TEST_ASSERT_NOT_NULL(curve->xpos);
    TEST_ASSERT_NOT_NULL(curve->ypos);

    TEST_ASSERT_FLOAT_WITHIN(0.0001, expected_controlx, controlx);
    TEST_ASSERT_FLOAT_WITHIN(0.0001, expected_controlz, controlz);
    TEST_ASSERT_FLOAT_WITHIN(0.0001, expected_endx, endx);
    TEST_ASSERT_FLOAT_WITHIN(0.0001, expected_endz, endz);
}

void test_calculate_swing_stance_phase_back(void)
{
    float startx = 97.86;
    float startz = 97.86;
    float stride_length = 50.0;
    float swing_height = 20.0;
    float endx, endz, controlx, controlz;
    bool is_swing_phase = true;
    struct bezier2d *curve = get_curve2d();

    calculate_swing_stance_phase(startx, startz, stride_length, swing_height, BACK, &controlx,
                                 &controlz, &endx, &endz, is_swing_phase);
    bezier2d_generate_curve(curve, startx, startz, controlx, controlz, endx, endz);

    float expected_controlx = startx - stride_length / 2;
    float expected_controlz = startz + 2 * swing_height;
    float expected_endx = startx - stride_length;
    float expected_endz = startz;

    TEST_ASSERT_NOT_NULL(curve->xpos);
    TEST_ASSERT_NOT_NULL(curve->ypos);

    TEST_ASSERT_FLOAT_WITHIN(0.0001, expected_controlx, controlx);
    TEST_ASSERT_FLOAT_WITHIN(0.0001, expected_controlz, controlz);
    TEST_ASSERT_FLOAT_WITHIN(0.0001, expected_endx, endx);
    TEST_ASSERT_FLOAT_WITHIN(0.0001, expected_endz, endz);
}

void test_generate_swing_phase_front(void)
{
    struct bezier2d *curve = get_curve2d();
    float startx = 97.86;
    float startz = 97.86;
    float stride_length = 50.0;
    float swing_height = 20.0;
    int leg_type = FRONT;

    generate_swing_phase(curve, startx, startz, stride_length, swing_height, leg_type);

    float expected_controlx = startx + stride_length / 2;
    float expected_controlz = startz + 2 * swing_height;
    float expected_endx = startx + stride_length;
    float expected_endz = startz;

    TEST_ASSERT_NOT_NULL(curve->xpos);
    TEST_ASSERT_NOT_NULL(curve->ypos);

    TEST_ASSERT_FLOAT_WITHIN(0.0001, expected_controlx, curve->xpos[1]);
    TEST_ASSERT_FLOAT_WITHIN(0.0001, expected_controlz, curve->ypos[1]);
    TEST_ASSERT_FLOAT_WITHIN(0.0001, expected_endx, curve->xpos[2]);
    TEST_ASSERT_FLOAT_WITHIN(0.0001, expected_endz, curve->ypos[2]);
}

void test_generate_swing_phase_back(void)
{
    struct bezier2d *curve = get_curve2d();
    float startx = 97.86;
    float startz = 97.86;
    float stride_length = 50.0;
    float swing_height = 20.0;
    int leg_type = BACK;

    generate_swing_phase(curve, startx, startz, stride_length, swing_height, leg_type);

    float expected_controlx = startx - stride_length / 2;
    float expected_controlz = startz + 2 * swing_height;
    float expected_endx = startx - stride_length;
    float expected_endz = startz;

    TEST_ASSERT_NOT_NULL(curve->xpos);
    TEST_ASSERT_NOT_NULL(curve->ypos);

    TEST_ASSERT_FLOAT_WITHIN(0.0001, expected_controlx, curve->xpos[1]);
    TEST_ASSERT_FLOAT_WITHIN(0.0001, expected_controlz, curve->ypos[1]);
    TEST_ASSERT_FLOAT_WITHIN(0.0001, expected_endx, curve->xpos[2]);
    TEST_ASSERT_FLOAT_WITHIN(0.0001, expected_endz, curve->ypos[2]);
}

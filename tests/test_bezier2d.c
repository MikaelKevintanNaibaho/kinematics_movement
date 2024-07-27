// bezier2d_tests.c
#include <unity/unity.h>
#include "test_bezier2d.h"
#include "common.h"

void test_bezier2d_init(void)
{
    struct bezier2d *curve = get_curve2d();
    TEST_ASSERT_NULL(curve->xpos);
    TEST_ASSERT_NULL(curve->ypos);
    TEST_ASSERT_EQUAL(0, curve->npoints);
}

void test_bezier2d_addPoint(void)
{
    struct bezier2d *curve = get_curve2d();
    bezier2d_addPoint(curve, 1.0, 1.0);
    TEST_ASSERT_NOT_NULL(curve->xpos);
    TEST_ASSERT_NOT_NULL(curve->ypos);
    TEST_ASSERT_EQUAL(1, curve->npoints);
    TEST_ASSERT_EQUAL_FLOAT(1.0, curve->xpos[0]);
    TEST_ASSERT_EQUAL_FLOAT(1.0, curve->ypos[0]);
}

void test_bezier2d_getPos(void)
{
    struct bezier2d *curve = get_curve2d();
    bezier2d_addPoint(curve, 0.0, 0.0);
    bezier2d_addPoint(curve, 1.0, 1.0);
    bezier2d_addPoint(curve, 2.0, 0.0);

    float xret, yret;
    bezier2d_getPos(curve, 0.5, &xret, &yret);

    TEST_ASSERT_EQUAL_FLOAT(1.0, xret); // Adjust if necessary
    TEST_ASSERT_EQUAL_FLOAT(0.5, yret); // Adjust if necessary
}

void test_bezier2d_generate_curve(void)
{
    struct bezier2d *curve = get_curve2d();
    bezier2d_generate_curve(curve, 0.0, 0.0, 1.0, 1.0, 2.0, 0.0);
    TEST_ASSERT_EQUAL(3, curve->npoints);
    TEST_ASSERT_EQUAL_FLOAT(0.0, curve->xpos[0]);
    TEST_ASSERT_EQUAL_FLOAT(1.0, curve->xpos[1]);
    TEST_ASSERT_EQUAL_FLOAT(2.0, curve->xpos[2]);
    TEST_ASSERT_EQUAL_FLOAT(0.0, curve->ypos[0]);
    TEST_ASSERT_EQUAL_FLOAT(1.0, curve->ypos[1]);
    TEST_ASSERT_EQUAL_FLOAT(0.0, curve->ypos[2]);
}
